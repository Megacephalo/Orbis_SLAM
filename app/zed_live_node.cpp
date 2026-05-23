#include <iostream>
#include <memory>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "orbis_slam/zed_setup_help_utils.h"
#include "orbis_slam/zed_wrapper.h"
#include "orbis_slam/orbis_slam_pipeline.h"


namespace Orbis {

/**
 * @brief ZED Live Stream - Handles live streaming from ZED camera and TF broadcasting
 *
 * This class manages the live stream from the ZED camera, including
 * initializing the camera, retrieving frames, and broadcasting TF transforms.
 */
class ZED_Live_Stream_node : public rclcpp::Node {
  private:
    Orbis::ZEDWrapper zed_wrapper_;

    // SLAM pipeline (unified with live camera mode)
    std::shared_ptr<Orbis::OrbisSLAMPipeline> slam_pipeline_;

    // ROS 2 node reference (provided by wrapper node)
    std::string world_frame_;
    std::string odom_frame_;
    std::string robot_baselink_frame_;
    std::string left_camera_frame_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;

    uint64_t current_frame_idx_;

  public:
    ZED_Live_Stream_node()
    : Node("zed_live_stream_node") {
        // Initialize TF broadcaster and listener
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Set parameters
        set_parameters();
    }
    ~ZED_Live_Stream_node() = default;

    bool initializeCamera();
    bool processFrame();
    void stopStream();

    int getCameraFPS() const;

    typedef std::shared_ptr<ZED_Live_Stream_node> Ptr;
    static ZED_Live_Stream_node::Ptr create() {
        return std::make_shared<ZED_Live_Stream_node>();
    }

  private:
    void set_parameters() {
        // Frame names
        this->declare_parameter<std::string>("world_frame", "world");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("robot_baselink_frame", "base_link");
        this->declare_parameter<std::string>("left_camera_frame", "zed_left_camera");

        this->get_parameter("world_frame", world_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("robot_baselink_frame", robot_baselink_frame_);
        this->get_parameter("left_camera_frame", left_camera_frame_);
    }




    void broadcastTF(
        const tf2::Transform& transf, 
        const std::string& parent_frame, 
        const std::string& child_frame, 
        const rclcpp::Time& timestamp) {

        geometry_msgs::msg::TransformStamped tf_msg;
        // tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.stamp = timestamp;
        tf_msg.header.frame_id = parent_frame;
        tf_msg.child_frame_id = child_frame;
        tf_msg.transform.translation.x = transf.getOrigin().x();
        tf_msg.transform.translation.y = transf.getOrigin().y();
        tf_msg.transform.translation.z = transf.getOrigin().z();
        tf_msg.transform.rotation.x = transf.getRotation().x();
        tf_msg.transform.rotation.y = transf.getRotation().y();
        tf_msg.transform.rotation.z = transf.getRotation().z();
        tf_msg.transform.rotation.w = transf.getRotation().w();

        tf_broadcaster_->sendTransform(tf_msg);
    }
};

} /* namespace Orbis */


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(Orbis::ZED_Live_Stream_node::create());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
