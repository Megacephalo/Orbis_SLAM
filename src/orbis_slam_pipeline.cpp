#include "orbis_slam/orbis_slam_pipeline.h"

namespace Orbis {

OrbisSLAMPipeline::OrbisSLAMPipeline(bool use_camera)
: use_camera_(use_camera)
, curr_frame_id_(0)
, last_keyframe_(nullptr)
{
    setupParameters();

    trajectory_ = Trajectory::create();
    pose_optimizer_.setTrajectory(trajectory_);

    // Initialize loop closure components (integral part of SLAM)
    if (enable_slam_) {
        // Create feature extractor
        feature_extractor_ = FeatureExtractor::create(1000, 1.2f, 8);

        // Create covisibility graph
        covisibility_graph_ = CovisibilityGraph::create(15);

        // Create loop closure detector
        loop_closure_detector_ = LoopClosureDetector::create(
            vocabulary_path_,
            covisibility_graph_,
            trajectory_,
            0.7,  // min similarity score
            30    // min temporal separation
        );

        // Set loop closure callback
        loop_closure_detector_->setOptimizationCallback(
            std::bind(&OrbisSLAMPipeline::onLoopClosureDetected, this, std::placeholders::_1)
        );

        // Start loop closure thread
        if (loop_closure_detector_->isReady()) {
            loop_closure_detector_->start();
            std::cout << "Loop closure detection started successfully." << std::endl;
        } else {
            std::cout << "Loop closure vocabulary not loaded. Loop closure will not be available." << std::endl;
        }
    }

    // Note: Timer creation is now handled by wrapper nodes (ZED_Live_Node)
    // The pipeline no longer manages its own timer since it's not a ROS 2 node
} /* ctor */

void
OrbisSLAMPipeline::setupParameters() {
    // Default parameters - these should be set by the wrapper node
    enable_slam_ = false;
    vocabulary_path_ = "";

    // Note: Parameters are now managed by wrapper nodes (ZED_Live_Node, ZED_Playback_Node)
    // The pipeline receives necessary configuration through constructor or setter methods
}

void
OrbisSLAMPipeline::processFrame() {
    // This method is deprecated and should not be called.
    // Frame processing is now handled by wrapper nodes (ZED_Live_Node, ZED_Playback_Node)
    // which call processSLAMFrame() directly with appropriate data.
    std::cerr << "WARNING: OrbisSLAMPipeline::processFrame() is deprecated. "
              << "Use processSLAMFrame() from wrapper nodes instead." << std::endl;
} /* processFrame */

bool
OrbisSLAMPipeline::processSLAMFrame(
    const Sophus::SE3d& T_w_c,
    const cv::Mat& left_image,
    double timestamp,
    const Sophus::SE3d& T_odom_leftCamera)
{
    // Note: TF broadcasting is now handled by wrapper nodes
    // This method focuses on pure SLAM processing

    if ( ! enable_slam_ ) {
        return false;
    }

    // Create current frame with the provided pose
    Frame::Ptr curr_frame = Frame::create(curr_frame_id_++, T_w_c, false, timestamp);

    keyframe_selector_.processFrame(curr_frame);

    if (! curr_frame->isKeyFrame() ) return false;

    // Extract features and 3D map points for loop closure (integral to SLAM)
    extractFeaturesAndMapPoints(curr_frame, left_image);

    trajectory_->pushBack(curr_frame);

    if (trajectory_->size() == 1) {
        last_keyframe_ = curr_frame;
        pose_optimizer_.addPoseVertex( curr_frame, true ); // fix the first keyframe at the origin

        // Add first keyframe to covisibility graph
        if (covisibility_graph_) {
            covisibility_graph_->addKeyframe(curr_frame->id);
        }
        return true;
    }

    pose_optimizer_.addPoseVertex( curr_frame, false);

    // Add the relative motion edge
    // Create a simple identity information matrix since we don't have ZED covariance here
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
    pose_optimizer_.addRelativeMotionEdgeFromPoses(last_keyframe_, curr_frame, information);

    // Update covisibility graph and loop closure detection
    if (covisibility_graph_ && loop_closure_detector_) {
        covisibility_graph_->addKeyframe(curr_frame->id);
        covisibility_graph_->updateCovisibility(
            last_keyframe_->id, curr_frame->id,
            last_keyframe_->map_points, curr_frame->map_points,
            last_keyframe_->valid_map_points, curr_frame->valid_map_points
        );

        // Add keyframe to loop closure detector queue
        loop_closure_detector_->addKeyframe(curr_frame);
    }

    if (pose_optimizer_.shouldOptimize(timestamp)) {
        pose_optimizer_.requestOptimization(timestamp);
    }

    // Update frame pose with optimized value
    curr_frame->pose = pose_optimizer_.getPose(curr_frame->id);

    // Note: TF broadcasting is handled by wrapper nodes
    // The optimized pose is stored in curr_frame->pose and can be retrieved by the wrapper

    last_keyframe_ = curr_frame;
    return true;
} /* processSLAMFrame */

void
OrbisSLAMPipeline::onLoopClosureDetected(const LoopClosureDetector::LoopCandidate& candidate) {
    std::cout << "Loop closure detected: KF " << candidate.query_keyframe_id
              << " -> KF " << candidate.match_keyframe_id
              << " (score: " << candidate.similarity_score << ")" << std::endl;

    // Add loop closure constraint to pose graph
    pose_optimizer_.addRelativeMotionEdge(
        candidate.match_keyframe_id,
        candidate.query_keyframe_id,
        candidate.relative_pose,
        candidate.information
    );

    // Trigger global optimization immediately
    // Use 0.0 as timestamp to force immediate optimization
    pose_optimizer_.requestOptimization(0.0);

    std::cout << "Global optimization triggered by loop closure." << std::endl;
}

void
OrbisSLAMPipeline::extractFeaturesAndMapPoints(Frame::Ptr frame, const cv::Mat& image) {
    if (!feature_extractor_ || image.empty()) {
        return;
    }

    // Extract ORB features
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    if (!feature_extractor_->extract(image, keypoints, descriptors)) {
        std::cerr << "WARNING: Failed to extract features from frame " << frame->id << std::endl;
        return;
    }

    // Note: 3D correspondences are now expected to be provided by wrapper nodes
    // or computed separately. This method only handles feature extraction.
    // For now, create placeholder map points (wrapper nodes should provide actual 3D data)
    std::vector<cv::Point3f> map_points_3d(keypoints.size(), cv::Point3f(0.0f, 0.0f, 0.0f));
    std::vector<bool> valid_flags(keypoints.size(), false);

    // Store in frame
    frame->keypoints = std::move(keypoints);
    frame->descriptors = descriptors.clone();
    frame->map_points = std::move(map_points_3d);
    frame->valid_map_points = std::move(valid_flags);

    // Note: Wrapper nodes should populate map_points and valid_map_points with actual 3D data
}
} /* namespace Orbis */