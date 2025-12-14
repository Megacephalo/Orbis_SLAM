#include "orbis_slam/loop_closure_detector.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

namespace Orbis {

LoopClosureDetector::LoopClosureDetector(
    const std::string& vocabulary_path,
    CovisibilityGraph::Ptr covisibility_graph,
    Trajectory::Ptr trajectory,
    double min_similarity,
    int min_temporal_sep
)
: vocabulary_loaded_(false)
, covisibility_graph_(covisibility_graph)
, trajectory_(trajectory)
, should_stop_(false)
, is_running_(false)
, min_similarity_score_(min_similarity)
, min_temporal_separation_(min_temporal_sep)
, min_covisibility_threshold_(15)
{
    // Initialize vocabulary manager
    vocabulary_manager_ = VocabularyManager::create(vocabulary_path);

    auto status = vocabulary_manager_->initialize();

    switch (status) {
        case VocabularyManager::Status::LOADED_FROM_FILE:
            std::cout << "[LoopClosureDetector] Using existing vocabulary" << std::endl;
            vocabulary_loaded_ = true;
            break;

        case VocabularyManager::Status::CREATED_ONLINE:
            std::cout << "[LoopClosureDetector] Will build vocabulary online from features" << std::endl;
            std::cout << "[LoopClosureDetector] Loop closure will become more accurate as vocabulary improves" << std::endl;
            vocabulary_loaded_ = true;  // Can start with minimal vocabulary
            break;

        case VocabularyManager::Status::FAILED:
            std::cerr << "[LoopClosureDetector] Failed to initialize vocabulary" << std::endl;
            vocabulary_loaded_ = false;
            return;
    }

    // Initialize database with the vocabulary
    try {
        database_.setVocabulary(vocabulary_manager_->getVocabulary(), false, 0);
        std::cout << "[LoopClosureDetector] Database initialized successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[LoopClosureDetector] Failed to initialize database: " << e.what() << std::endl;
        vocabulary_loaded_ = false;
    }
}

LoopClosureDetector::~LoopClosureDetector() {
    stop();
}

void LoopClosureDetector::start() {
    if (is_running_ || !vocabulary_loaded_) {
        return;
    }

    should_stop_ = false;
    is_running_ = true;
    processing_thread_ = std::thread(&LoopClosureDetector::processingLoop, this);
    std::cout << "[LoopClosureDetector] Started loop closure detection thread." << std::endl;
}

void LoopClosureDetector::stop() {
    if (!is_running_) {
        return;
    }

    should_stop_ = true;
    queue_cv_.notify_all();

    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }

    is_running_ = false;
    std::cout << "[LoopClosureDetector] Stopped loop closure detection thread." << std::endl;
}

void LoopClosureDetector::addKeyframe(Frame::Ptr keyframe) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    keyframe_queue_.push(keyframe);
    queue_cv_.notify_one();
}

void LoopClosureDetector::setOptimizationCallback(std::function<void(const LoopCandidate&)> callback) {
    optimization_callback_ = callback;
}

std::vector<LoopClosureDetector::LoopCandidate> LoopClosureDetector::getLoopCandidates() const {
    std::lock_guard<std::mutex> lock(candidates_mutex_);
    return loop_candidates_;
}

void LoopClosureDetector::processingLoop() {
    while (!should_stop_) {
        Frame::Ptr keyframe = nullptr;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !keyframe_queue_.empty() || should_stop_;
            });

            if (should_stop_) {
                break;
            }

            if (!keyframe_queue_.empty()) {
                keyframe = keyframe_queue_.front();
                keyframe_queue_.pop();
            }
        }

        if (keyframe) {
            processKeyframe(keyframe);
        }
    }
}

void LoopClosureDetector::processKeyframe(Frame::Ptr keyframe) {
    if (!keyframe || keyframe->descriptors.empty()) {
        return;
    }

    // Add features to vocabulary manager for online learning
    if (vocabulary_manager_) {
        vocabulary_manager_->addTrainingFeatures(keyframe->descriptors);
    }

    // Convert cv::Mat descriptors to DBoW3 format
    std::vector<cv::Mat> descriptor_vector;
    descriptor_vector.reserve(keyframe->descriptors.rows);
    for (int i = 0; i < keyframe->descriptors.rows; ++i) {
        descriptor_vector.push_back(keyframe->descriptors.row(i));
    }

    // Detect loop candidates
    auto candidates = detectLoopCandidates(keyframe);

    // Process each candidate
    for (auto& candidate : candidates) {
        Frame::Ptr match_frame = trajectory_->findPtrById(candidate.match_keyframe_id);
        if (!match_frame) {
            continue;
        }

        // Perform geometric verification
        if (geometricVerification(candidate, keyframe, match_frame)) {
            candidate.is_verified = true;

            // Store verified candidate
            {
                std::lock_guard<std::mutex> lock(candidates_mutex_);
                loop_candidates_.push_back(candidate);
            }

            std::cout << "[LoopClosureDetector] Loop detected! Query KF: " << candidate.query_keyframe_id
                      << " -> Match KF: " << candidate.match_keyframe_id
                      << " (score: " << candidate.similarity_score << ")" << std::endl;

            // Trigger optimization callback
            if (optimization_callback_) {
                optimization_callback_(candidate);
            }
        }
    }

    // Add current keyframe to database
    DBoW3::BowVector bow_vector;
    DBoW3::FeatureVector feature_vector;
    vocabulary_manager_->getVocabulary().transform(descriptor_vector, bow_vector, feature_vector, 4);  // 4 = levelsup
    database_.add(bow_vector, feature_vector);
}

std::vector<LoopClosureDetector::LoopCandidate> LoopClosureDetector::detectLoopCandidates(
    Frame::Ptr keyframe
) {
    std::vector<LoopCandidate> candidates;

    if (database_.size() < static_cast<size_t>(min_temporal_separation_)) {
        return candidates;  // Not enough history yet
    }

    // Convert descriptors to DBoW3 format
    std::vector<cv::Mat> descriptor_vector;
    descriptor_vector.reserve(keyframe->descriptors.rows);
    for (int i = 0; i < keyframe->descriptors.rows; ++i) {
        descriptor_vector.push_back(keyframe->descriptors.row(i));
    }

    // Query the database
    DBoW3::QueryResults query_results;
    database_.query(descriptor_vector, query_results, 5);  // Get top 5 matches

    // Filter results
    for (const auto& result : query_results) {
        // result.Id is the database entry ID, which corresponds to keyframe insertion order
        // We need to map this to actual keyframe IDs
        // For simplicity, we assume database entry ID matches keyframe order in trajectory
        // A more robust implementation would maintain an explicit mapping

        uint64_t match_id = result.Id;  // This assumes sequential insertion

        // Check temporal separation
        if (static_cast<int64_t>(keyframe->id) - static_cast<int64_t>(match_id) < min_temporal_separation_) {
            continue;
        }

        // Check similarity score
        if (result.Score < min_similarity_score_) {
            continue;
        }

        // Check that frames are not already covisible (avoid trivial loops)
        int covisibility_weight = covisibility_graph_->getWeight(keyframe->id, match_id);
        if (covisibility_weight > min_covisibility_threshold_) {
            continue;  // Already strongly connected
        }

        // Create candidate
        LoopCandidate candidate;
        candidate.query_keyframe_id = keyframe->id;
        candidate.match_keyframe_id = match_id;
        candidate.similarity_score = result.Score;
        candidate.is_verified = false;

        candidates.push_back(candidate);
    }

    return candidates;
}

bool LoopClosureDetector::geometricVerification(
    LoopCandidate& candidate,
    Frame::Ptr query_frame,
    Frame::Ptr match_frame
) {
    // Match features between the two frames using descriptor matching
    cv::BFMatcher matcher(cv::NORM_HAMMING, true);  // true = cross-check
    std::vector<cv::DMatch> matches;

    if (query_frame->descriptors.empty() || match_frame->descriptors.empty()) {
        return false;
    }

    matcher.match(query_frame->descriptors, match_frame->descriptors, matches);

    // Filter matches by distance
    if (matches.size() < 20) {
        return false;  // Too few matches
    }

    // Sort matches by distance
    std::sort(matches.begin(), matches.end(),
        [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        }
    );

    // Keep only good matches (best 70%)
    size_t num_good_matches = static_cast<size_t>(matches.size() * 0.7);
    matches.resize(num_good_matches);

    if (matches.size() < 20) {
        return false;
    }

    // Extract 3D point correspondences
    std::vector<cv::Point3f> points_query;
    std::vector<cv::Point3f> points_match;

    for (const auto& m : matches) {
        int query_idx = m.queryIdx;
        int match_idx = m.trainIdx;

        if (query_idx >= 0 && query_idx < static_cast<int>(query_frame->valid_map_points.size()) &&
            match_idx >= 0 && match_idx < static_cast<int>(match_frame->valid_map_points.size())) {

            if (query_frame->valid_map_points[query_idx] &&
                match_frame->valid_map_points[match_idx]) {

                points_query.push_back(query_frame->map_points[query_idx]);
                points_match.push_back(match_frame->map_points[match_idx]);
            }
        }
    }

    if (points_query.size() < 15) {
        return false;  // Not enough 3D correspondences
    }

    // Compute relative pose
    return computeRelativePose(query_frame, match_frame, candidate.relative_pose, candidate.information);
}

bool LoopClosureDetector::computeRelativePose(
    Frame::Ptr frame1,
    Frame::Ptr frame2,
    Sophus::SE3d& relative_pose,
    Eigen::Matrix<double, 6, 6>& information
) {
    // Use the stored poses from the frames
    // Relative pose: T_1_2 = T_w_1^{-1} * T_w_2
    relative_pose = frame1->pose.inverse() * frame2->pose;

    // Set information matrix
    // Higher confidence for loop closures that pass geometric verification
    information = Eigen::Matrix<double, 6, 6>::Identity();
    information.block<3, 3>(0, 0) *= 100.0;  // Translation
    information.block<3, 3>(3, 3) *= 100.0;  // Rotation

    return true;
}

} /* namespace Orbis */
