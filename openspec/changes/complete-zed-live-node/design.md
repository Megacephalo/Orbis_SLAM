## Context

`ZED_Live_Stream_node` (in `app/zed_live_node.cpp`) is a ROS 2 node that wraps `ZEDWrapper` and `OrbisSLAMPipeline`. It currently declares `initializeCamera()`, `processFrame()`, `stopStream()`, and `getCameraFPS()` but none are implemented. The working reference is `ZED_Playback_Node` (`app/zed_playback_node.cpp`), which follows the pattern: constructor does all setup → wall timer drives `processFrame()` → `processSLAMFrame()` feeds the pipeline. The ZED SDK helper `toSophus(sl::Pose&)` already exists in `zed_setup_help_utils.h`, so no new conversion code is needed.

## Goals / Non-Goals

**Goals:**
- Constructor-complete initialization (camera open, pipeline init, publishers, timer) — no external `initializeCamera()` call required
- Timer-driven `processFrame()` that mirrors the playback node: grab → convert pose → broadcast TF → publish images → feed SLAM pipeline
- `broadcastTF` unified to `Sophus::SE3d` signature (eliminating the `tf2::Transform` variant)
- CMake parity: `orbis_slam_node` gets the same dependency set as `orbis_slam_playback_node`

**Non-Goals:**
- IMU fusion or sensor data beyond left image and pose
- Depth image publishing (point cloud retrieval already happens inside `grabFrame()` for 3D correspondences, but depth topic publication is deferred)
- Lifecycle node management
- Right image publishing in this change

## Decisions

**D1: Inline initialization into constructor, remove public `initializeCamera()`**

The playback node does all setup in the constructor and throws `std::runtime_error` on failure. ROS 2's `rclcpp::spin` propagates that exception and exits cleanly. Keeping `initializeCamera()` as a separate public method would require callers to remember to invoke it — a footgun with no upside here.

Alternatives considered: keep as public method called from `main()`. Rejected: adds indirection with no benefit; `main()` already works as a one-liner.

**D2: Use `this->get_clock()->now()` for TF timestamps, ZED timestamp for SLAM**

`sl::Pose::timestamp` may be 0 on the first few frames before tracking locks. TF with a stale/zero timestamp causes downstream lookup failures. Using wall-clock time for TF (same as playback node) avoids this. The SLAM pipeline's `timestamp` argument uses the ZED SDK nanosecond timestamp converted to seconds, which is the authoritative sensor time.

**D3: `broadcastTF` takes `Sophus::SE3d`, not `tf2::Transform`**

The `tf2::Transform` variant in the current stub is inconsistent with `Sophus::SE3d` used throughout the rest of the pipeline. Switching to Sophus eliminates an unnecessary type boundary and makes the live/playback implementations read identically.

**D4: Timer interval = `1000ms / getCameraFPS()`**

Camera FPS is queried after `setup()` succeeds. This matches the playback node's approach of computing timer interval from the data source's native rate.

**D5: `slam_pipeline_` constructed with `use_camera = true`**

Playback uses `false` (headless). Live mode passes `true` so the pipeline can initialize camera-specific subsystems if needed in future. Currently `OrbisSLAMPipeline` ignores this flag internally, but the semantic distinction should be preserved.

## Risks / Trade-offs

`grabFrame()` blocks until `hasNewFrame()` returns true — this is called from the timer callback on the ROS executor thread. If the ZED SDK ever stalls, the timer callback will block and starve other callbacks.
→ Mitigation: acceptable for now given single-node deployment; a dedicated grab thread can be introduced if latency issues emerge.

`toCvMat()` returns a `cv::Mat` that shares memory with the `sl::Mat` buffer. If `grabFrame()` is called again before the SLAM pipeline finishes with the image, the buffer will be overwritten.
→ Mitigation: `OrbisSLAMPipeline::processSLAMFrame()` is synchronous, so the buffer is safe within a single timer callback. Document this assumption.

## Open Questions

- Should depth images be published in this change, or deferred? (Current recommendation: defer — keeps scope tight.)
- Does `OrbisSLAMPipeline(true)` need to do anything special for live mode, or is the flag purely future-proofing?
