# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### Standard Build
```bash
# Build from ROS 2 workspace root
cd ~/ros2_ws
colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### Single-threaded Build (for limited memory systems like Jetson)
```bash
colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 1
```

### Clean Build
```bash
rm -rf build/orbis_slam install/orbis_slam log/orbis_slam
colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running the System

### Main SLAM Node
```bash
# Direct execution
ros2 run orbis_slam orbis_slam_node

# With launch file (sensor-only mode)
ros2 launch orbis_slam orbis_slam_sensor_only.py
```

### ZED Recorder/Player Tool
```bash
ros2 run orbis_slam ZEDRecPlay
```

## Architecture Overview

### Core Pipeline Structure
The system follows a modular pipeline architecture coordinated by `OrbisSLAMPipeline`:

1. **ZEDWrapper** (`zed_wrapper.h`) - Camera interface layer
   - Initializes and manages ZED SDK camera
   - Provides calibration data and frame capture
   - Abstracts hardware specifics from SLAM logic

2. **KeyFrameSelector** (`keyframe_selector.h`) - Keyframe decision logic
   - Determines when to create new keyframes based on motion
   - Manages frame-to-frame tracking
   - Critical for map size and optimization performance

3. **PoseOptimizer** (`pose_optimizer.h`) - Backend optimization
   - Uses Ceres Solver for non-linear least squares optimization
   - Uses g2o for pose graph optimization
   - Performs bundle adjustment on keyframes

4. **Essential Data Structures** (`essential_data_structure.h`)
   - `Frame`: Represents a single camera frame with SE3 pose (Sophus)
   - `Trajectory`: Thread-safe container for frame history
   - Uses Sophus SE3/SO3 for Lie algebra-based transformations

### Data Flow
```
ZED Camera → ZEDWrapper → OrbisSLAMPipeline → KeyFrameSelector
                                    ↓
                            PoseOptimizer (if keyframe)
                                    ↓
                            TF2 Broadcaster (world→odom→base_link)
```

### Transform Frames
- `map` (world_frame): Global reference frame
- `odom` (odom_frame): Odometry frame
- `base_link` (robot_baselink_frame): Robot base
- Camera frame (left_camera_frame): Left camera optical frame

The node publishes transforms: `map→odom→base_link→camera`

### Thread Safety
The `Trajectory` class uses reader-writer locks (`std::shared_mutex`) for concurrent access to pose history. Multiple readers can access simultaneously, but writes are exclusive.

## Code Organization

### Main Libraries
- `Orbis_SLAM_pipeline` (src/CMakeLists.txt): Core library containing pipeline, optimizer, and keyframe selector
- Links against: ZED SDK, OpenCV, Eigen3, Sophus, g2o, Ceres, TBB, fmt, nlohmann_json

### Executables
1. `orbis_slam_node`: Main SLAM node (`app/orbis_slam_main.cpp`)
2. `ZEDRecPlay`: Data recording/playback tool with Qt6 GUI (`app/ZEDRecPlay.cpp`)

### Third-Party Modules (in third_parties/)
- Ceres Solver: Non-linear optimization
- Sophus: Lie algebra operations (SE3, SO3)
- g2o: Graph-based optimization
- OctoMap: Included but not yet integrated

## Key Parameters

ROS 2 parameters (configurable via launch files):
- `world_frame`: Global reference frame (default: "map")
- `odom_frame`: Odometry frame (default: "odom")
- `robot_baselink_frame`: Robot base frame (default: "base_link")
- `left_camera_frame`: Camera optical frame
- `enable_slam`: Enable/disable backend optimization (true/false)

When `enable_slam=false`, the system operates in pure odometry mode without optimization.

## Dependencies

### System Libraries
- Eigen3 (linear algebra)
- TBB (parallel computing)
- FMT (string formatting)
- OpenCV (computer vision)
- nlohmann_json (JSON parsing)
- Qt6 (for ZEDRecPlay GUI: Core, Gui, OpenGL, Widgets, OpenGLWidgets)
- GLM (OpenGL mathematics)

### Third-Party (as submodules)
- Ceres Solver (build from third_party/ceres-solver)
- Sophus (build from third_party/Sophus)
- g2o (system-installed or build from source)

### Hardware
- ZED SDK (required, version matched with CUDA)
- CUDA-capable GPU

## Development Notes

### C++ Standard
The project uses C++17 features. All code must compile with `-std=c++17`.

### Optimization Backend
Two optimization frameworks are used:
- **Ceres Solver**: For non-linear least squares problems
- **g2o**: For pose graph optimization with various solver backends

Sophus provides SE(3)/SO(3) parameterizations compatible with both frameworks.

### ROS 2 Integration
- Built for ROS 2 Humble
- Uses ament_cmake build system
- Integrates with tf2 for coordinate transformations
- Standard ROS 2 message types: sensor_msgs, geometry_msgs, nav_msgs

### Recorder/Player System
The ZEDRecPlay tool provides:
- Record ZED camera data streams to disk
- Playback recorded sequences for testing
- Qt6-based GUI for control
- Useful for development without camera hardware

### Current Branch
The repository is on branch `record-and-play-back-a-datast` with recent work on:
- ZED recorder/player implementation
- Technical documentation
- Third-party module integration (Ceres, Sophus)

### Code Style
- Google C++ Style Guide
- Meaningful variable/function names
- Comprehensive comments for complex algorithms
- Modern C++17 idioms
