#ifndef _LOOP_CLOSURE_DETECTOR_H_
#define _LOOP_CLOSURE_DETECTOR_H_

#include <memory>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>

#include <DBoW3/DBoW3.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "orbis_slam/essential_data_structure.h"
#include "orbis_slam/covisibility_graph.h"
#include "orbis_slam/vocabulary_manager.h"

namespace Orbis {

/**
 * @brief LoopClosureDetector detects loop closures using DBoW3 vocabulary
 *
 * This class manages loop closure detection by:
 * 1. Maintaining a DBoW3 database of keyframe descriptors
 * 2. Detecting potential loop closures using place recognition
 * 3. Computing geometric verification and relative pose constraints
 * 4. Signaling the main thread to trigger global optimization
 */
class LoopClosureDetector {
public:
    struct LoopCandidate {
        uint64_t query_keyframe_id;      // Current keyframe
        uint64_t match_keyframe_id;      // Matched historical keyframe
        double similarity_score;          // DBoW3 similarity score
        Sophus::SE3d relative_pose;      // Relative pose constraint
        Eigen::Matrix<double, 6, 6> information;  // Information matrix
        bool is_verified;                 // Geometric verification passed

        LoopCandidate()
            : query_keyframe_id(0)
            , match_keyframe_id(0)
            , similarity_score(0.0)
            , relative_pose(Sophus::SE3d())
            , information(Eigen::Matrix<double, 6, 6>::Identity())
            , is_verified(false)
        {}
    };

private:
    // DBoW3 vocabulary and database
    VocabularyManager::Ptr vocabulary_manager_;
    DBoW3::Database database_;
    bool vocabulary_loaded_;

    // Covisibility graph reference
    CovisibilityGraph::Ptr covisibility_graph_;

    // Trajectory reference for accessing keyframes
    Trajectory::Ptr trajectory_;

    // Thread-safe keyframe queue
    std::queue<Frame::Ptr> keyframe_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    // Loop closure processing thread
    std::thread processing_thread_;
    std::atomic<bool> should_stop_;
    std::atomic<bool> is_running_;

    // Loop closure candidates (protected by mutex)
    std::vector<LoopCandidate> loop_candidates_;
    mutable std::mutex candidates_mutex_;

    // Detection parameters
    double min_similarity_score_;     // Minimum DBoW3 similarity score
    int min_temporal_separation_;     // Minimum keyframe ID difference
    int min_covisibility_threshold_;  // Minimum covisibility for geometric check

    // Callback for signaling optimization
    std::function<void(const LoopCandidate&)> optimization_callback_;

public:
    /**
     * @brief Construct a new Loop Closure Detector
     *
     * @param vocabulary_path Path to DBoW3 vocabulary file
     * @param covisibility_graph Shared covisibility graph
     * @param trajectory Shared trajectory
     * @param min_similarity Minimum similarity score for loop detection (default: 0.7)
     * @param min_temporal_sep Minimum keyframe separation to avoid recent frames (default: 30)
     */
    LoopClosureDetector(
        const std::string& vocabulary_path,
        CovisibilityGraph::Ptr covisibility_graph,
        Trajectory::Ptr trajectory,
        double min_similarity = 0.7,
        int min_temporal_sep = 30
    );

    ~LoopClosureDetector();

    // Non-copyable
    LoopClosureDetector(const LoopClosureDetector&) = delete;
    LoopClosureDetector& operator=(const LoopClosureDetector&) = delete;

    /**
     * @brief Start the loop closure detection thread
     */
    void start();

    /**
     * @brief Stop the loop closure detection thread
     */
    void stop();

    /**
     * @brief Add a keyframe to the processing queue
     *
     * @param keyframe Keyframe to process
     */
    void addKeyframe(Frame::Ptr keyframe);

    /**
     * @brief Set callback function for optimization trigger
     *
     * The callback is called when a verified loop closure is detected,
     * allowing the main thread to trigger global optimization.
     *
     * @param callback Function to call with loop closure candidate
     */
    void setOptimizationCallback(std::function<void(const LoopCandidate&)> callback);

    /**
     * @brief Get all detected loop candidates
     *
     * @return std::vector<LoopCandidate> List of loop candidates
     */
    std::vector<LoopCandidate> getLoopCandidates() const;

    /**
     * @brief Check if vocabulary is loaded
     *
     * @return true if vocabulary is ready
     */
    bool isReady() const { return vocabulary_loaded_; }

    typedef std::shared_ptr<LoopClosureDetector> Ptr;

    static LoopClosureDetector::Ptr create(
        const std::string& vocabulary_path,
        CovisibilityGraph::Ptr covisibility_graph,
        Trajectory::Ptr trajectory,
        double min_similarity = 0.7,
        int min_temporal_sep = 30
    ) {
        return std::make_shared<LoopClosureDetector>(
            vocabulary_path, covisibility_graph, trajectory, min_similarity, min_temporal_sep
        );
    }

private:
    /**
     * @brief Main processing loop for loop closure detection
     */
    void processingLoop();

    /**
     * @brief Process a single keyframe for loop closure detection
     *
     * @param keyframe Keyframe to process
     */
    void processKeyframe(Frame::Ptr keyframe);

    /**
     * @brief Detect loop closure candidates using DBoW3
     *
     * @param keyframe Query keyframe
     * @return std::vector<LoopCandidate> Potential loop candidates
     */
    std::vector<LoopCandidate> detectLoopCandidates(Frame::Ptr keyframe);

    /**
     * @brief Perform geometric verification on a loop candidate
     *
     * @param candidate Loop candidate to verify
     * @param query_frame Query keyframe
     * @param match_frame Matched keyframe
     * @return true if geometric verification passed
     */
    bool geometricVerification(
        LoopCandidate& candidate,
        Frame::Ptr query_frame,
        Frame::Ptr match_frame
    );

    /**
     * @brief Compute relative pose between two frames using feature matches
     *
     * @param frame1 First frame
     * @param frame2 Second frame
     * @param relative_pose Output relative pose
     * @param information Output information matrix
     * @return true if pose estimation succeeded
     */
    bool computeRelativePose(
        Frame::Ptr frame1,
        Frame::Ptr frame2,
        Sophus::SE3d& relative_pose,
        Eigen::Matrix<double, 6, 6>& information
    );
};

} /* namespace Orbis */

#endif /* _LOOP_CLOSURE_DETECTOR_H_ */
