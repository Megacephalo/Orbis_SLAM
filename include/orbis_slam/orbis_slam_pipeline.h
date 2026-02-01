 
 
 #ifndef _ORBIS_SLAM_PIPELINE_H_
#define _ORBIS_SLAM_PIPELINE_H_

#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>


#include "orbis_slam/essential_data_structure.h"
#include "orbis_slam/keyframe_selector.h"
#include "orbis_slam/pose_optimizer.h"
#include "orbis_slam/feature_extractor.h"
#include "orbis_slam/covisibility_graph.h"
#include "orbis_slam/loop_closure_detector.h"

namespace Orbis {

/**
 * @brief Core SLAM pipeline library - does NOT inherit from ROS 2 Node
 *
 * This is a pure C++ library that performs the core SLAM processing.
 * ROS 2 wrapper nodes (ZED_Live_Node, ZED_Playback_Node) manage the ROS 2
 * lifecycle and call this library for SLAM processing.
 */
class OrbisSLAMPipeline {
  private:
    Orbis::KeyFrameSelector keyframe_selector_;
    Orbis::PoseOptimizer pose_optimizer_;

    bool enable_slam_;
    bool use_camera_;  // Whether to use ZED camera hardware or run headless
    uint64_t curr_frame_id_;
    Frame::Ptr last_keyframe_;
    Trajectory::Ptr trajectory_;

    // Loop closure components (integral part of SLAM)
    FeatureExtractor::Ptr feature_extractor_;
    CovisibilityGraph::Ptr covisibility_graph_;
    LoopClosureDetector::Ptr loop_closure_detector_;
    std::string vocabulary_path_;

  public:
    /**
     * @brief Construct OrbisSLAMPipeline
     * @param node ROS 2 node pointer for logging, parameters, and TF
     * @param use_camera If true, initialize ZED camera hardware (default: true)
     *                   If false, run in headless mode for playback
     */
    explicit OrbisSLAMPipeline(bool use_camera = true);
    ~OrbisSLAMPipeline() = default;  // WIP: for now

    void processFrame(); 

    /**
     * @brief Process a SLAM frame from any source (camera or playback)
     *
     * This method decouples SLAM processing from frame acquisition, allowing
     * both live camera data and recorded playback data to be processed.
     *
     * @param pose Camera pose (T_world_camera) from odometry/tracking
     * @param left_image Left camera image
     * @param timestamp Frame timestamp in seconds
     * @param T_odom_leftCamera Transform from odom to left camera (for TF publishing)
     * @return true if frame was processed as a keyframe, false otherwise
     */
    bool processSLAMFrame(
        const Sophus::SE3d& pose,
        const cv::Mat& left_image,
        double timestamp,
        const Sophus::SE3d& T_odom_leftCamera
    );

    typedef std::shared_ptr<OrbisSLAMPipeline> Ptr;
    static OrbisSLAMPipeline::Ptr create(bool use_camera = true) {
        return std::make_shared<OrbisSLAMPipeline>(use_camera);
    }

  private:
    void setupParameters();

    /**
     * @brief Callback for loop closure detection
     *
     * This is called by the loop closure detector when a verified loop is found.
     * It adds the loop constraint to the pose graph and triggers optimization.
     *
     * @param candidate Loop closure candidate with relative pose constraint
     */
    void onLoopClosureDetected(const LoopClosureDetector::LoopCandidate& candidate);

    /**
     * @brief Extract features and 3D map points for a frame
     *
     * @param frame Frame to process
     * @param image Left camera image
     */
    void extractFeaturesAndMapPoints(Frame::Ptr frame, const cv::Mat& image);
}; /* class OrbisSLAMPipeline */

} /* namespace Orbis */
#endif /* _ORBIS_SLAM_PIPELINE_H_ */