#ifndef _ORBIS_SLAM_PIPELINE_H_
#define _ORBIS_SLAM_PIPELINE_H_

#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "orbis_slam/zed_setup_help_utils.h"
#include "orbis_slam/zed_wrapper.h"
#include "orbis_slam/essential_data_structure.h"
#include "orbis_slam/keyframe_selector.h"
#include "orbis_slam/pose_optimizer.h"

namespace Orbis {

class OrbisSLAMPipeline : public rclcpp::Node {
  private:
    Orbis::ZEDWrapper zed_wrapper_;
    Orbis::KeyFrameSelector keyframe_selector_;
    Orbis::PoseOptimizer pose_optimizer_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string world_frame_;
    std::string odom_frame_;
    std::string robot_baselink_frame_;
    std::string left_camera_frame_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool enable_slam_;
    uint64_t curr_frame_id_;
    Frame::Ptr last_keyframe_;
    Trajectory::Ptr trajectory_;

  public:
    OrbisSLAMPipeline();
    ~OrbisSLAMPipeline() = default;  // WIP: for now

    void processFrame();
    
    typedef std::shared_ptr<OrbisSLAMPipeline> Ptr;
    static OrbisSLAMPipeline::Ptr create() {
        return std::make_shared<OrbisSLAMPipeline>();
    }
  
  private:
    void setupParameters();
    void broadcastTF(const tf2::Transform& transf, const std::string& parent_frame, const std::string& child_frame, const rclcpp::Time& timestamp);
}; /* class OrbisSLAMPipeline */

} /* namespace Orbis */
#endif /* _ORBIS_SLAM_PIPELINE_H_ */