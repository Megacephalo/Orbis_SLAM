#include <iostream>
#include <memory>
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

#include "zed_recording.pb.h"
#include "orbis_slam/essential_data_structure.h"
#include "orbis_slam/keyframe_selector.h"
#include "orbis_slam/pose_optimizer.h"
#include "orbis_slam/orbis_slam_pipeline.h"

namespace Orbis {

/**
 * @brief ZED Playback Node - Reads protobuf recordings and runs them through SLAM pipeline
 *
 * This node reads ZED camera recordings stored in protobuf format and processes them
 * through the Orbis SLAM pipeline, publishing the results to ROS 2 topics and TF frames
 * for visualization in RViz2.
 */
class ZED_Playback_Node : public rclcpp::Node {
private:
    // SLAM pipeline (unified with live camera mode)
    std::shared_ptr<Orbis::OrbisSLAMPipeline> slam_pipeline_;

    // ROS 2 infrastructure
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;

    // Playback state
    std::unique_ptr<orbis_slam::Recording> recording_;
    uint64_t current_frame_idx_;
    bool playback_active_;
    double playback_rate_;
    bool enable_loop_;

    // File path
    std::string recording_file_path_;

    // TF frame names
    std::string world_frame_;
    std::string odom_frame_;
    std::string robot_baselink_frame_;
    std::string left_camera_frame_;

public:
    ZED_Playback_Node()
    : Node("zed_playback_node")
    , current_frame_idx_(0)
    , playback_active_(false)
    , enable_loop_(false)
    {
        RCLCPP_INFO(this->get_logger(), "ZED Playback Node initializing...");

        // Setup parameters
        setupParameters();

        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Load the recording file
        if (!loadRecording()) {
            throw std::runtime_error("Failed to load recording file: " + recording_file_path_);
        }

        // Create publishers
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/left/image_raw", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/right/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/depth/image_raw", 10);

        // Initialize SLAM pipeline in headless mode (without ZED camera hardware)
        slam_pipeline_ = std::make_shared<Orbis::OrbisSLAMPipeline>(false);  // false = no camera

        // Calculate playback timer interval based on recording FPS and playback rate
        double fps = recording_->fps() > 0 ? recording_->fps() : 30.0;
        int timer_interval_ms = static_cast<int>(1000.0 / (fps * playback_rate_));

        // Create timer for frame processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms),
            std::bind(&ZED_Playback_Node::processFrame, this)
        );

        playback_active_ = true;
        RCLCPP_INFO(this->get_logger(), "ZED Playback Node initialized successfully.");
        RCLCPP_INFO(this->get_logger(), "Loaded recording with %d frames at %.1f FPS",
                    recording_->frames_size(), fps);
    }

    typedef std::shared_ptr<ZED_Playback_Node> Ptr;
    static ZED_Playback_Node::Ptr create() {
        return std::make_shared<ZED_Playback_Node>();
    }

private:
    void setupParameters() {
        // Playback parameters
        this->declare_parameter<std::string>("recording_file", "");
        this->declare_parameter<double>("playback_rate", 1.0);
        this->declare_parameter<bool>("enable_loop", false);

        // TF frame parameters
        this->declare_parameter<std::string>("world_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("robot_baselink_frame", "base_link");
        this->declare_parameter<std::string>("left_camera_frame", "zed_left_camera_frame");

        // Get parameters
        recording_file_path_ = this->get_parameter("recording_file").as_string();
        playback_rate_ = this->get_parameter("playback_rate").as_double();

        // Handle enable_loop - LaunchConfiguration passes strings, so handle both types
        auto loop_param = this->get_parameter("enable_loop");
        if (loop_param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            enable_loop_ = loop_param.as_bool();
        } else if (loop_param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            std::string loop_str = loop_param.as_string();
            enable_loop_ = (loop_str == "true" || loop_str == "True" || loop_str == "1");
        } else {
            enable_loop_ = false;
        }
        RCLCPP_INFO(this->get_logger(), "Looping playback: %s", enable_loop_ ? "enabled" : "disabled");
        world_frame_ = this->get_parameter("world_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        robot_baselink_frame_ = this->get_parameter("robot_baselink_frame").as_string();
        left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();

        // Validate recording file path
        if (recording_file_path_.empty()) {
            throw std::runtime_error("Recording file path not specified. Set 'recording_file' parameter.");
        }

        // Validate playback rate
        if (playback_rate_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid playback_rate %.2f, setting to 1.0", playback_rate_);
            playback_rate_ = 1.0;
        }

        RCLCPP_INFO(this->get_logger(), "TF Frames: %s -> %s -> %s -> %s",
                    world_frame_.c_str(), odom_frame_.c_str(),
                    robot_baselink_frame_.c_str(), left_camera_frame_.c_str());
    }

    bool loadRecording() {
        RCLCPP_INFO(this->get_logger(), "Loading recording from: %s", recording_file_path_.c_str());

        std::ifstream input(recording_file_path_, std::ios::binary);
        if (!input.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open recording file: %s",
                        recording_file_path_.c_str());
            return false;
        }

        recording_ = std::make_unique<orbis_slam::Recording>();
        if (!recording_->ParseFromIstream(&input)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf recording from file");
            return false;
        }

        input.close();

        RCLCPP_INFO(this->get_logger(), "Successfully loaded recording:");
        RCLCPP_INFO(this->get_logger(), "  Recording ID: %s", recording_->recording_id().c_str());
        RCLCPP_INFO(this->get_logger(), "  Frames: %d", recording_->frames_size());
        RCLCPP_INFO(this->get_logger(), "  Camera Model: %s", recording_->camera_model().c_str());
        RCLCPP_INFO(this->get_logger(), "  Description: %s", recording_->description().c_str());

        return true;
    }

    void processFrame() {
        if (!rclcpp::ok() || !playback_active_) {
            return;
        }

        // Check if we've reached the end of the recording
        if (current_frame_idx_ >= static_cast<uint64_t>(recording_->frames_size())) {
            if (enable_loop_) {
                RCLCPP_INFO(this->get_logger(), "Looping playback. Restarting from frame 0.");
                current_frame_idx_ = 0;
            } else {
                RCLCPP_INFO(this->get_logger(), "Playback complete. Processed %lu frames.",
                           current_frame_idx_);
                playback_active_ = false;
                timer_->cancel();
                return;
            }
        }

        // Get current frame from recording
        const auto& pb_frame = recording_->frames(current_frame_idx_);

        // Create timestamp - use current time for TF to avoid stale transform issues
        double timestamp = pb_frame.timestamp_ns() / 1e9;  // Convert nanoseconds to seconds
        rclcpp::Time ros_time = this->get_clock()->now();  // Use current time for real-time playback

        // Publish images to ROS topics
        publishImages(pb_frame, ros_time);

        // Check if pose data is available in the recording
        if (!pb_frame.has_pose()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Frame %lu has no pose data", current_frame_idx_);
            current_frame_idx_++;
            return;
        }

        const auto& pb_pose = pb_frame.pose();

        // Debug: Log pose for first few frames
        if (current_frame_idx_ < 3) {
            RCLCPP_INFO(this->get_logger(), "Frame %lu pose: t=[%.3f, %.3f, %.3f] q=[%.3f, %.3f, %.3f, %.3f]",
                current_frame_idx_,
                pb_pose.tx(), pb_pose.ty(), pb_pose.tz(),
                pb_pose.qx(), pb_pose.qy(), pb_pose.qz(), pb_pose.qw());
        }

        // Extract pose from recording (world/map to camera transform)
        Eigen::Vector3d translation(pb_pose.tx(), pb_pose.ty(), pb_pose.tz());
        Eigen::Quaterniond quaternion(pb_pose.qw(), pb_pose.qx(), pb_pose.qy(), pb_pose.qz());
        quaternion.normalize();
        Sophus::SE3d T_world_camera(quaternion, translation);

        // Convert left image for SLAM processing
        cv::Mat left_image;
        if (pb_frame.has_left_image()) {
            auto left_msg = decompressImage(pb_frame.left_image(), ros_time, left_camera_frame_);
            if (left_msg) {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
                left_image = cv_ptr->image;
            }
        }

        // Broadcast TF transforms: map -> odom -> camera_base_link
        // robot_state_publisher handles: camera_base_link -> ... -> left_camera_optical_frame
        if (current_frame_idx_ < 3) {
            RCLCPP_INFO(this->get_logger(), "Broadcasting TF: %s -> %s and %s -> %s",
                world_frame_.c_str(), odom_frame_.c_str(),
                odom_frame_.c_str(), robot_baselink_frame_.c_str());
        }
        broadcastTF(Sophus::SE3d(), world_frame_, odom_frame_, ros_time);
        broadcastTF(T_world_camera, odom_frame_, robot_baselink_frame_, ros_time);

        // Process frame through SLAM pipeline if we have an image
        if (!left_image.empty()) {
            // Create T_odom_leftCamera for SLAM (same as T_world_camera for playback)
            slam_pipeline_->processSLAMFrame(T_world_camera, left_image, timestamp, T_world_camera);
        }

        // Log progress periodically
        if (current_frame_idx_ % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "Processing frame %lu/%d (%.1f%%)",
                       current_frame_idx_, recording_->frames_size(),
                       100.0 * current_frame_idx_ / recording_->frames_size());
        }

        // Move to next frame
        current_frame_idx_++;
    }

    void broadcastTF(const Sophus::SE3d& transform,
                     const std::string& parent_frame,
                     const std::string& child_frame,
                     const rclcpp::Time& timestamp) {
        if (!tf_broadcaster_) {
            RCLCPP_ERROR(this->get_logger(), "TF broadcaster is null!");
            return;
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = timestamp;
        tf_msg.header.frame_id = parent_frame;
        tf_msg.child_frame_id = child_frame;

        // Translation
        Eigen::Vector3d trans = transform.translation();
        tf_msg.transform.translation.x = trans.x();
        tf_msg.transform.translation.y = trans.y();
        tf_msg.transform.translation.z = trans.z();

        // Rotation
        Eigen::Quaterniond quat = transform.unit_quaternion();
        tf_msg.transform.rotation.x = quat.x();
        tf_msg.transform.rotation.y = quat.y();
        tf_msg.transform.rotation.z = quat.z();
        tf_msg.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void publishImages(const orbis_slam::Frame& pb_frame, const rclcpp::Time& timestamp) {
        // Publish left image
        if (pb_frame.has_left_image()) {
            auto left_msg = decompressImage(pb_frame.left_image(), timestamp, left_camera_frame_);
            if (left_msg) {
                left_image_pub_->publish(*left_msg);
            }
        }

        // Publish right image
        if (pb_frame.has_right_image()) {
            auto right_msg = decompressImage(pb_frame.right_image(), timestamp, left_camera_frame_);
            if (right_msg) {
                right_image_pub_->publish(*right_msg);
            }
        }

        // Publish depth image
        if (pb_frame.has_depth_image()) {
            auto depth_msg = decompressImage(pb_frame.depth_image(), timestamp, left_camera_frame_, true);
            if (depth_msg) {
                depth_image_pub_->publish(*depth_msg);
            }
        }
    }

    sensor_msgs::msg::Image::SharedPtr decompressImage(
        const orbis_slam::CompressedImage& compressed_img,
        const rclcpp::Time& timestamp,
        const std::string& frame_id,
        bool is_depth = false)
    {
        cv::Mat image;

        // Decode based on format
        switch (compressed_img.format()) {
            case orbis_slam::CompressedImage::JPEG:
            case orbis_slam::CompressedImage::PNG:
            case orbis_slam::CompressedImage::WEBP: {
                std::vector<uint8_t> data(compressed_img.data().begin(), compressed_img.data().end());
                image = cv::imdecode(data, is_depth ? cv::IMREAD_UNCHANGED : cv::IMREAD_COLOR);
                break;
            }
            case orbis_slam::CompressedImage::RAW: {
                // Raw uncompressed data
                int cv_type;
                if (is_depth) {
                    cv_type = CV_16UC1;  // Typical depth format
                } else {
                    cv_type = compressed_img.channels() == 1 ? CV_8UC1 :
                             compressed_img.channels() == 3 ? CV_8UC3 : CV_8UC4;
                }
                image = cv::Mat(compressed_img.height(), compressed_img.width(), cv_type,
                               const_cast<char*>(compressed_img.data().data())).clone();
                break;
            }
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown image format");
                return nullptr;
        }

        if (image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to decompress image");
            return nullptr;
        }

        // Convert to ROS message
        std_msgs::msg::Header header;
        header.stamp = timestamp;
        header.frame_id = frame_id;

        std::string encoding;
        if (is_depth) {
            encoding = "16UC1";
        } else {
            encoding = image.channels() == 1 ? "mono8" : "bgr8";
        }

        return cv_bridge::CvImage(header, encoding, image).toImageMsg();
    }
};

} /* namespace Orbis */


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(Orbis::ZED_Playback_Node::create());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}