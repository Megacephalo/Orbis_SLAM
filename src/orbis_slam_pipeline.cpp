#include "orbis_slam/orbis_slam_pipeline.h"

namespace Orbis {

OrbisSLAMPipeline::OrbisSLAMPipeline()
: Node("orbis_slam_node")
, curr_frame_id_(0)
, last_keyframe_(nullptr)
{
    bool camera_ready = zed_wrapper_.setup();
    if ( ! camera_ready ) {
        throw std::runtime_error("Camera setup failed. Exit.");
    }

    setupParameters();

    // initialize the TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    camera_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cam_pc_topic_name_, 10);
    left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(left_image_topic_name_, 10);
    stereo_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(stereo_image_topic_name_, 10);

    trajectory_ = Trajectory::create();
    pose_optimizer_.setTrajectory(trajectory_);

    // create a timer to process frame at the requested frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / zed_wrapper_.getCameraFPS()),
        std::bind(&OrbisSLAMPipeline::processFrame, this)
    );
} /* ctor */

void
OrbisSLAMPipeline::setupParameters() {
    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("robot_baselink_frame", "base_link");
    this->declare_parameter<std::string>("left_camera_frame", "left_camera_frame");
    this->declare_parameter<bool>("enable_slam", false);
    this->declare_parameter<std::string>("cam_center_frame", "camera_center");
    this->declare_parameter<std::string>("cam_pointcloud_topic", "zed/point_cloud");
    this->declare_parameter<std::string>("left_image_topic", "zed/left_image");
    this->declare_parameter<std::string>("stereo_image_topic", "zed/stereo_image");

    world_frame_ = this->get_parameter("world_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    robot_baselink_frame_ = this->get_parameter("robot_baselink_frame").as_string();
    left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();
    enable_slam_ = this->get_parameter("enable_slam").as_bool();
    cam_center_frame_ = this->get_parameter("cam_center_frame").as_string();
    cam_pc_topic_name_ = this->get_parameter("cam_pointcloud_topic").as_string();
    left_image_topic_name_ = this->get_parameter("left_image_topic").as_string();
    stereo_image_topic_name_ = this->get_parameter("stereo_image_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "Parameters set:");
    RCLCPP_INFO(this->get_logger(), "    world_frame:           %s", world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "    odom_frame:            %s", odom_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "    robot_baselink_frame:  %s", robot_baselink_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "    left_camera_frame:     %s", left_camera_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "    cam_center_frame:      %s", cam_center_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "    cam_pointcloud_topic:  %s", cam_pc_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "    enable_slam:           %s", enable_slam_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "    left_image_topic:      %s", left_image_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "    stereo_image_topic:    %s", stereo_image_topic_name_.c_str());
}

void
OrbisSLAMPipeline::processFrame() {
    if ( ! rclcpp::ok() ) {
        RCLCPP_INFO(this->get_logger(), "ROS2 halted. Exit.");
        return;
    }
    
    if ( ! zed_wrapper_.hasNewFrame() ) {
        RCLCPP_WARN(this->get_logger(), "No new frame from camera.");
        return;
    }

    zed_wrapper_.grabFrame();

    // Publish odometry pose as TF
    // step 1: T_odom_left_camera
    sl::Pose T_w_c = zed_wrapper_.sl_getPose();
    auto curr_translation = T_w_c.getTranslation();
    auto curr_orientation = T_w_c.getOrientation();

    tf2::Transform T_odom_leftCamera;
    T_odom_leftCamera.setOrigin(tf2::Vector3(
        curr_translation.x,
        curr_translation.y,
        curr_translation.z
    ));
    T_odom_leftCamera.setRotation(tf2::Quaternion(
        curr_orientation.x,
        curr_orientation.y,
        curr_orientation.z,
        curr_orientation.w
    ));
    // apply rotation: ZED to ROS camera frame conversion
    tf2::Quaternion zed_to_ros_rotation;
    tf2::Quaternion zed_quaternion(
        curr_orientation.x,
        curr_orientation.y,
        curr_orientation.z,
        curr_orientation.w
    );
    zed_to_ros_rotation.setRPY(-M_PI / 2, 0, -M_PI / 2); // Rotate to align ZED with ROS camera convention
    // Combine the ZED orientation with the coordinate frame conversion
    T_odom_leftCamera.setRotation(zed_quaternion * zed_to_ros_rotation);


    // step 2: T_robot_leftCamera
    geometry_msgs::msg::TransformStamped T_robot_leftCamera_msg;
    try {
        T_robot_leftCamera_msg = tf_buffer_->lookupTransform(
            robot_baselink_frame_,  // target frame
            left_camera_frame_,    // source frame
            tf2::TimePointZero,      // get the latest available
            tf2::durationFromSec(1.0)   // wait for 1 second
        );
    }
    catch(const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform from %s to %s: %s",
            robot_baselink_frame_.c_str(),
            left_camera_frame_.c_str(),
            ex.what());
        return; // skip this frame and try again on the next one
    }
    
    tf2::Transform T_robot_leftCamera;
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

    // step 3: T_odom_robot = T_odom_left_camera * T_robot_leftCamera.inverse()
    rclcpp::Time current_time = this->get_clock()->now();

    tf2::Transform T_odom_robot = T_odom_leftCamera * T_robot_leftCamera.inverse();
    broadcastTF(T_odom_robot, odom_frame_, robot_baselink_frame_, current_time);

    // Publish images and point cloud
    left_image_pub_->publish(*convertCvMatToRosImage(zed_wrapper_.getLeftImage(), left_camera_frame_, current_time));
    stereo_image_pub_->publish(*convertCvMatToRosImage(zed_wrapper_.getStereoImage(), cam_center_frame_, current_time));  

    //// // See https://github.com/stereolabs/zed-sdk/blob/e9ab2621552a2e0a2ea37a0352133c45e8162f7b/tutorials/tutorial%203%20-%20depth%20sensing/cpp/main.cpp#L57
    auto ros2_cloud = convertZEDToPointCloud2(zed_wrapper_.sl_getPointCloud(), left_camera_frame_, current_time);
    camera_pointcloud_pub_->publish(ros2_cloud);

    if ( ! enable_slam_ ) {
        tf2::Transform T_map_odom;
        T_map_odom.setIdentity();
        broadcastTF(T_map_odom, world_frame_, odom_frame_, current_time);
        return;
    }

    // TODO: get the T_prev_curr_ and optimize in terms of pose-graph
    Frame::Ptr curr_frame = Frame::create(curr_frame_id_++, toSophus(T_w_c), false, current_time.seconds());

    keyframe_selector_.processFrame(curr_frame);

    if (! curr_frame->isKeyFrame() ) return;

    trajectory_->pushBack(curr_frame);

    if (trajectory_->size() == 1) {
        last_keyframe_ = curr_frame;
        pose_optimizer_.addPoseVertex( curr_frame, true ); // fix the first keyframe at the origin
        return;
    }
    
    pose_optimizer_.addPoseVertex( curr_frame, false);

    // // add the relative motion edge
    Eigen::Matrix<double, 6, 6> information = PoseOptimizer::createInformationMatrix(T_w_c);
    pose_optimizer_.addRelativeMotionEdgeFromPoses(last_keyframe_, curr_frame, information);

    if (pose_optimizer_.shouldOptimize(current_time.seconds())) {
        pose_optimizer_.requestOptimization(current_time.seconds());
    }

    curr_frame->pose = pose_optimizer_.getPose(curr_frame->id);



    // Publish the optimized pose as TF
    tf2::Transform T_map_cam;
    auto optimized_translation = curr_frame->pose.translation();
    T_map_cam.setOrigin( tf2::Vector3(
        optimized_translation.x(),
        optimized_translation.y(),
        optimized_translation.z()
    ));
    Eigen::Quaterniond q = curr_frame->pose.unit_quaternion();
    T_map_cam.setRotation( tf2::Quaternion(
        q.x(),
        q.y(),
        q.z(),
        q.w()
    ));
    tf2::Transform T_map_odom = T_map_cam * T_odom_robot.inverse();
    broadcastTF(T_map_odom, world_frame_, odom_frame_, this->get_clock()->now());

    // TODO: Publish the global map as point cloud message
    // TODO: Publish the left and right images as ROS2 image messages

    last_keyframe_ = curr_frame;
} /* processFrame */

void
OrbisSLAMPipeline::broadcastTF(const tf2::Transform& transf, const std::string& parent_frame, const std::string& child_frame, const rclcpp::Time& timestamp) {
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

sensor_msgs::msg::PointCloud2
convertZEDToPointCloud2(const sl::Mat& zed_cloud, const std::string& frame_id, const rclcpp::Time& timestamp) {    
    sensor_msgs::msg::PointCloud2 cloud_msg;

    // Set header
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = timestamp;

    // Get cloud dimensions
    int width = zed_cloud.getWidth();
    int height = zed_cloud.getHeight();
    
    cloud_msg.width = width;
    cloud_msg.height = height;
    cloud_msg.is_dense = false;  // May contain NaN values
    
    // Define fields: X, Y, Z, RGBA
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgba");

    // create iterators
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(cloud_msg, "a");

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            sl::float4 point3D;
            zed_cloud.getValue(x, y, &point3D);

            // Set coordinates (use NaN for invalid points instead of skipping)
            if (std::isfinite(point3D.x) && std::isfinite(point3D.y) && std::isfinite(point3D.z) && 
                point3D.x != 0.0f && point3D.y != 0.0f && point3D.z != 0.0f) {
                *iter_x = point3D.x;
                *iter_y = point3D.y;
                *iter_z = point3D.z;

                // Correct ARGB unpacking (ZED uses ARGB format)
                uint32_t argb = *reinterpret_cast<uint32_t*>(&point3D.w);
                *iter_a = (argb >> 24) & 0xFF;  // A
                *iter_r = (argb >> 16) & 0xFF;  // R
                *iter_g = (argb >> 8) & 0xFF;   // G
                *iter_b = argb & 0xFF;          // B

            } else {
                // Set invalid points to NaN
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
                *iter_r = 0;
                *iter_g = 0;
                *iter_b = 0;
                *iter_a = 0;
            }

            // Fixed: semicolon instead of comma
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b; ++iter_a;
        }
    }

    return cloud_msg;
}

sensor_msgs::msg::Image::SharedPtr
convertCvMatToRosImage(const cv::Mat& image, const std::string& frame_id, const rclcpp::Time& timestamp) {
    if (image.empty()) {
        throw std::runtime_error("Input cv::Mat image is empty.");
    }

    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = timestamp;
    cv_image.header.frame_id = frame_id;

    // Determine encoding based on image properties
    std::string encoding;
    if (image.channels() == 1) {
        encoding = "mono8";
    } else if (image.channels() == 3) {
        encoding = "bgr8";
    } else if (image.channels() == 4) {
        encoding = "bgra8";
    } else {
        throw std::runtime_error("Unsupported number of channels: " + std::to_string(image.channels()));
    }
    cv_image.encoding = encoding;

    // Ensure the image is in the correct format
    cv::Mat processed_image;
    if (image.type() == CV_8UC3) {
        processed_image = image;
    } else if (image.type() == CV_8UC1) {
        processed_image = image;
    } else if (image.type() == CV_8UC4) {
        processed_image = image;
    } else {
        // Convert to 8-bit if needed
        image.convertTo(processed_image, CV_8UC3);
    }
    cv_image.image = processed_image;

    return cv_image.toImageMsg();
}

} /* namespace Orbis */