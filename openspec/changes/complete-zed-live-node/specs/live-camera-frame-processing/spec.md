## ADDED Requirements

### Requirement: Node initializes camera and pipeline in constructor
The `ZED_Live_Stream_node` constructor SHALL open the ZED camera via `ZEDWrapper::setup()`, construct an `OrbisSLAMPipeline` with `use_camera=true`, create image publishers, and register a wall timer at the camera's native FPS. If camera initialization fails, the constructor SHALL throw `std::runtime_error`.

#### Scenario: Successful initialization
- **WHEN** `ZED_Live_Stream_node` is constructed and the ZED camera is available
- **THEN** the camera opens, publishers are created, and the wall timer starts firing at camera FPS

#### Scenario: Camera unavailable at startup
- **WHEN** `ZED_Live_Stream_node` is constructed and `ZEDWrapper::setup()` returns false
- **THEN** the constructor throws `std::runtime_error` and the node does not start

### Requirement: Timer callback grabs and processes each live frame
On each timer tick, the node SHALL call `ZEDWrapper::hasNewFrame()`. If a new frame is available, it SHALL call `ZEDWrapper::grabFrame()`, retrieve the left image via `ZEDWrapper::getLeftImage()`, convert the ZED pose to `Sophus::SE3d` via `toSophus()`, broadcast TF transforms, publish the left image, and call `slam_pipeline_->processSLAMFrame()`.

#### Scenario: New frame available
- **WHEN** the timer fires and `ZEDWrapper::hasNewFrame()` returns true
- **THEN** the frame is grabbed, pose is converted, TF is broadcast, image is published, and `processSLAMFrame()` is called

#### Scenario: No new frame available
- **WHEN** the timer fires and `ZEDWrapper::hasNewFrame()` returns false
- **THEN** the callback returns early without calling `grabFrame()` or `processSLAMFrame()`

### Requirement: TF broadcast uses wall-clock time and Sophus::SE3d
The node SHALL broadcast two transforms on every processed frame: identity (`map → odom`) and the camera pose (`odom → base_link`). TF timestamps SHALL use `this->get_clock()->now()`. The `broadcastTF` helper SHALL accept `Sophus::SE3d`, not `tf2::Transform`.

#### Scenario: TF published each frame
- **WHEN** a frame is successfully grabbed
- **THEN** `map → odom` (identity) and `odom → base_link` (camera pose) transforms are published with the current wall-clock time

### Requirement: getCameraFPS delegates to ZEDWrapper
`ZED_Live_Stream_node::getCameraFPS()` SHALL return `zed_wrapper_.getCameraFPS()`.

#### Scenario: FPS query after initialization
- **WHEN** `getCameraFPS()` is called after successful construction
- **THEN** the value returned matches the ZED camera's configured frame rate

### Requirement: stopStream cancels the timer
`ZED_Live_Stream_node::stopStream()` SHALL cancel the wall timer, stopping further frame processing.

#### Scenario: Stream stopped
- **WHEN** `stopStream()` is called
- **THEN** the wall timer is cancelled and no further `processFrame` callbacks fire
