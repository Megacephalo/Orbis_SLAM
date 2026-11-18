## Why

`zed_live_node.cpp` contains a `ZED_Live_Stream_node` class with three stubbed methods (`initializeCamera`, `processFrame`, `stopStream`) and a mismatched `broadcastTF` signature — meaning the live camera path is non-functional. The playback node already works end-to-end; the live node should mirror that pattern so both paths are operational.

## What Changes

- Move camera initialization logic into the constructor (removing the need for a public `initializeCamera()` method), mirroring the playback node's constructor pattern
- Implement `processFrame()` using `ZEDWrapper::hasNewFrame()` + `grabFrame()`, converting `sl::Pose` to `Sophus::SE3d` via the existing `toSophus()` helper, then calling `slam_pipeline_->processSLAMFrame()`
- Implement `stopStream()` to cancel the timer
- Implement `getCameraFPS()` to delegate to `ZEDWrapper::getCameraFPS()`
- Update `broadcastTF` signature from `tf2::Transform` to `Sophus::SE3d`, matching the playback node
- Fix `app/CMakeLists.txt`: add missing ament dependencies and link libraries for `orbis_slam_node`

## Capabilities

### New Capabilities
- `live-camera-frame-processing`: Continuous frame grab → pose conversion → SLAM pipeline ingestion loop driven by a ROS 2 wall timer at camera FPS

### Modified Capabilities
- (none — no spec-level requirement changes)

## Impact

- `app/zed_live_node.cpp`: primary file, substantive changes
- `app/CMakeLists.txt`: add ament deps (`sensor_msgs`, `geometry_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`) and link (`${OpenCV_LIBS}`) for `orbis_slam_node`
- No API or message-type changes; no impact on playback node
