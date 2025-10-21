#include "orbis_slam/orbis_slam_pipeline.h"

namespace Orbis {

OrbisSLAMPipeline::OrbisSLAMPipeline()
: Node("orbis_slam_pipeline_node")
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

    world_frame_ = this->get_parameter("world_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    robot_baselink_frame_ = this->get_parameter("robot_baselink_frame").as_string();
    left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();
    enable_slam_ = this->get_parameter("enable_slam").as_bool();
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
    T_robot_leftCamera_msg = tf_buffer_->lookupTransform(
        robot_baselink_frame_,  // target frame
        left_camera_frame_,    // source frame
        tf2::TimePointZero,      // get the latest available
        tf2::durationFromSec(1.0)   // wait for 1 second
    );
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

} /* namespace Orbis */