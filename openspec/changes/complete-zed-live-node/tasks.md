## 1. Fix CMake dependencies

- [ ] 1.1 Add `sensor_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge` to `ament_target_dependencies` for `orbis_slam_node` in `app/CMakeLists.txt`
- [ ] 1.2 Add `${OpenCV_LIBS}` to `target_link_libraries` for `orbis_slam_node` in `app/CMakeLists.txt`

## 2. Update includes and broadcastTF signature

- [ ] 2.1 Add missing includes to `zed_live_node.cpp`: `cv_bridge/cv_bridge.h`, `opencv2/opencv.hpp`, `sophus/se3.hpp`
- [ ] 2.2 Replace the `broadcastTF(tf2::Transform, ...)` private method with the `Sophus::SE3d` variant matching `zed_playback_node.cpp`

## 3. Implement constructor setup (replace public initializeCamera)

- [ ] 3.1 In the constructor body, call `zed_wrapper_.setup()` and throw `std::runtime_error` on failure
- [ ] 3.2 Construct `slam_pipeline_` with `OrbisSLAMPipeline(true)` (use_camera = true)
- [ ] 3.3 Create the three image publishers (`~/left/image_raw`, `~/right/image_raw`, `~/depth/image_raw`)
- [ ] 3.4 Create the wall timer at `1000ms / getCameraFPS()` bound to `processFrame`
- [ ] 3.5 Remove the now-dead `initializeCamera()` declaration from the public interface

## 4. Implement processFrame

- [ ] 4.1 Return early if `!zed_wrapper_.hasNewFrame()`
- [ ] 4.2 Call `zed_wrapper_.grabFrame()`
- [ ] 4.3 Capture `ros_time = this->get_clock()->now()` for TF
- [ ] 4.4 Extract timestamp from `zed_wrapper_.sl_getPose().timestamp.getNanoseconds() / 1e9` for SLAM
- [ ] 4.5 Convert ZED pose to `Sophus::SE3d` via `toSophus(zed_wrapper_.sl_getPose())`
- [ ] 4.6 Broadcast `map → odom` (identity) and `odom → base_link` (camera pose) TF transforms
- [ ] 4.7 Get left image via `zed_wrapper_.getLeftImage()` and publish to `left_image_pub_`
- [ ] 4.8 Call `slam_pipeline_->processSLAMFrame(T_world_camera, left_image, timestamp, T_world_camera)`
- [ ] 4.9 Increment `current_frame_idx_`

## 5. Implement stopStream and getCameraFPS

- [ ] 5.1 Implement `stopStream()`: cancel the timer (`timer_->cancel()`)
- [ ] 5.2 Implement `getCameraFPS()`: return `zed_wrapper_.getCameraFPS()`

## 6. Build verification

- [ ] 6.1 Run `colcon build --packages-select orbis_slam` and confirm zero errors and zero warnings related to `zed_live_node`
