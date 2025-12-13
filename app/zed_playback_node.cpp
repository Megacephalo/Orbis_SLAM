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

#include "zed_recording.pb.h"
#include "orbis_slam/essential_data_structure.h"
#include "orbis_slam/keyframe_selector.h"
#include "orbis_slam/pose_optimizer.h"

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
    // SLAM pipeline components
    Orbis::KeyFrameSelector keyframe_selector_;
    Orbis::PoseOptimizer pose_optimizer_;
    Orbis::Trajectory::Ptr trajectory_;

    // ROS 2 infrastructure
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;

    // Frame parameters
    std::string world_frame_;
    std::string odom_frame_;
    std::string robot_baselink_frame_;
    std::string left_camera_frame_;
    bool enable_slam_;

    // Playback state
    std::unique_ptr<orbis_slam::Recording> recording_;
    uint64_t current_frame_idx_;
    uint64_t curr_frame_id_;
    Frame::Ptr last_keyframe_;
    bool playback_active_;
    double playback_rate_;

    // File path
    std::string recording_file_path_;

public:
    ZED_Playback_Node()
    : Node("zed_playback_node")
    , current_frame_idx_(0)
    , curr_frame_id_(0)
    , last_keyframe_(nullptr)
    , playback_active_(false)
    {
        RCLCPP_INFO(this->get_logger(), "ZED Playback Node initializing...");

        // Setup parameters
        setupParameters();

        // Load the recording file
        if (!loadRecording()) {
            throw std::runtime_error("Failed to load recording file: " + recording_file_path_);
        }

        // Initialize TF broadcaster and listener
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize trajectory and pose optimizer
        trajectory_ = Trajectory::create();
        pose_optimizer_.setTrajectory(trajectory_);

        // Create publishers
        left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/left/image_raw", 10);
        right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/right/image_raw", 10);
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/depth/image_raw", 10);

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
        // Frame names
        this->declare_parameter<std::string>("world_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("robot_baselink_frame", "base_link");
        this->declare_parameter<std::string>("left_camera_frame", "left_camera_frame");

        // SLAM enable
        this->declare_parameter<bool>("enable_slam", true);

        // Playback parameters
        this->declare_parameter<std::string>("recording_file", "");
        this->declare_parameter<double>("playback_rate", 1.0);

        // Get parameters
        world_frame_ = this->get_parameter("world_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        robot_baselink_frame_ = this->get_parameter("robot_baselink_frame").as_string();
        left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();
        enable_slam_ = this->get_parameter("enable_slam").as_bool();
        recording_file_path_ = this->get_parameter("recording_file").as_string();
        playback_rate_ = this->get_parameter("playback_rate").as_double();

        // Validate recording file path
        if (recording_file_path_.empty()) {
            throw std::runtime_error("Recording file path not specified. Set 'recording_file' parameter.");
        }

        // Validate playback rate
        if (playback_rate_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid playback_rate %.2f, setting to 1.0", playback_rate_);
            playback_rate_ = 1.0;
        }
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
            RCLCPP_INFO(this->get_logger(), "Playback complete. Processed %lu frames.",
                       current_frame_idx_);
            playback_active_ = false;
            timer_->cancel();
            return;
        }

        // Get current frame from recording
        const orbis_slam::Frame& pb_frame = recording_->frames(current_frame_idx_);

        // Convert timestamp to ROS time
        rclcpp::Time current_time = rclcpp::Time(pb_frame.timestamp_ns());

        // Publish images
        publishImages(pb_frame, current_time);

        // Process pose through SLAM pipeline
        processPose(pb_frame, current_time);

        // Advance to next frame
        current_frame_idx_++;
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
            auto right_msg = decompressImage(pb_frame.right_image(), timestamp, "right_camera_frame");
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

    void processPose(const orbis_slam::Frame& pb_frame, const rclcpp::Time& current_time) {
        if (!pb_frame.has_pose()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Frame %lu has no pose data", current_frame_idx_);
            return;
        }

        const orbis_slam::CameraPose& pb_pose = pb_frame.pose();

        // Convert protobuf pose to Sophus SE3
        Eigen::Vector3d translation(pb_pose.tx(), pb_pose.ty(), pb_pose.tz());
        Eigen::Quaterniond quaternion(pb_pose.qw(), pb_pose.qx(), pb_pose.qy(), pb_pose.qz());
        quaternion.normalize();
        Sophus::SE3d pose(quaternion, translation);

        // Convert to tf2::Transform for broadcasting
        tf2::Transform T_odom_leftCamera;
        T_odom_leftCamera.setOrigin(tf2::Vector3(translation.x(), translation.y(), translation.z()));
        T_odom_leftCamera.setRotation(tf2::Quaternion(quaternion.x(), quaternion.y(),
                                                       quaternion.z(), quaternion.w()));

        // Apply ZED to ROS camera frame conversion (same as in orbis_slam_pipeline.cpp)
        tf2::Quaternion zed_to_ros_rotation;
        zed_to_ros_rotation.setRPY(-M_PI / 2, 0, -M_PI / 2);
        T_odom_leftCamera.setRotation(
            tf2::Quaternion(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w())
            * zed_to_ros_rotation
        );

        // Lookup transform from robot base to left camera
        tf2::Transform T_robot_leftCamera;
        try {
            geometry_msgs::msg::TransformStamped T_robot_leftCamera_msg;
            T_robot_leftCamera_msg = tf_buffer_->lookupTransform(
                robot_baselink_frame_,
                left_camera_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0)
            );
            T_robot_leftCamera.setOrigin(tf2::Vector3(
                T_robot_leftCamera_msg.transform.translation.x,
                T_robot_leftCamera_msg.transform.translation.y,
                T_robot_leftCamera_msg.transform.translation.z
            ));
            T_robot_leftCamera.setRotation(tf2::Quaternion(
                T_robot_leftCamera_msg.transform.rotation.x,
                T_robot_leftCamera_msg.transform.rotation.y,
                T_robot_leftCamera_msg.transform.rotation.z,
                T_robot_leftCamera_msg.transform.rotation.w
            ));
        } catch (const tf2::TransformException& ex) {
            // If transform not available, use identity
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "Could not get transform %s -> %s: %s. Using identity.",
                               robot_baselink_frame_.c_str(), left_camera_frame_.c_str(), ex.what());
            T_robot_leftCamera.setIdentity();
        }

        // Compute T_odom_robot = T_odom_leftCamera * T_robot_leftCamera.inverse()
        tf2::Transform T_odom_robot = T_odom_leftCamera * T_robot_leftCamera.inverse();
        broadcastTF(T_odom_robot, odom_frame_, robot_baselink_frame_, current_time);

        // Process through SLAM pipeline if enabled
        if (!enable_slam_) {
            // Just publish identity for map->odom
            tf2::Transform T_map_odom;
            T_map_odom.setIdentity();
            broadcastTF(T_map_odom, world_frame_, odom_frame_, current_time);
            return;
        }

        // Create frame for SLAM pipeline
        Frame::Ptr curr_frame = Frame::create(curr_frame_id_++, pose, false, current_time.seconds());

        // Process through keyframe selector
        keyframe_selector_.processFrame(curr_frame);

        if (!curr_frame->isKeyFrame()) {
            return;
        }

        // Add to trajectory
        trajectory_->pushBack(curr_frame);

        // First keyframe - fix at origin
        if (trajectory_->size() == 1) {
            last_keyframe_ = curr_frame;
            pose_optimizer_.addPoseVertex(curr_frame, true);
            return;
        }

        // Add new keyframe to optimizer
        pose_optimizer_.addPoseVertex(curr_frame, false);

        // Add relative motion edge
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
        information.block<3, 3>(0, 0) *= 100.0;  // Position information
        information.block<3, 3>(3, 3) *= 100.0;  // Rotation information
        pose_optimizer_.addRelativeMotionEdgeFromPoses(last_keyframe_, curr_frame, information);

        // Request optimization if needed
        if (pose_optimizer_.shouldOptimize(current_time.seconds())) {
            pose_optimizer_.requestOptimization(current_time.seconds());
        }

        // Get optimized pose
        curr_frame->pose = pose_optimizer_.getPose(curr_frame->id);

        // Publish optimized pose as TF
        tf2::Transform T_map_cam;
        auto optimized_translation = curr_frame->pose.translation();
        T_map_cam.setOrigin(tf2::Vector3(
            optimized_translation.x(),
            optimized_translation.y(),
            optimized_translation.z()
        ));
        Eigen::Quaterniond q = curr_frame->pose.unit_quaternion();
        T_map_cam.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));

        tf2::Transform T_map_odom = T_map_cam * T_odom_robot.inverse();
        broadcastTF(T_map_odom, world_frame_, odom_frame_, current_time);

        last_keyframe_ = curr_frame;
    }

    void broadcastTF(const tf2::Transform& transf,
                    const std::string& parent_frame,
                    const std::string& child_frame,
                    const rclcpp::Time& timestamp)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
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

    try {
        rclcpp::spin(Orbis::ZED_Playback_Node::create());
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}