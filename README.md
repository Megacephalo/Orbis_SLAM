<div align="center">

![Orbis SLAM Logo](documentation/orbis_logo_cropped.jpeg)

# Orbis SLAM

**A Modern Visual-Inertial Stereo SLAM System for ROS 2**

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen)](https://github.com/Megacephalo/Orbis_SLAM)

</div>

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Configuration](#configuration)
- [Architecture](#architecture)
- [Roadmap](#roadmap)
- [Contributing](#contributing)
- [License](#license)

## Overview

Orbis SLAM is a modern, real-time visual-inertial stereo Simultaneous Localization and Mapping (SLAM) system designed for ROS 2 and Stereolabs' ZED cameras. Built with performance and accuracy in mind, Orbis SLAM leverages state-of-the-art computer vision algorithms and optimization techniques to provide robust localization and mapping capabilities for robotic applications, building upon the excellent ZED SDK.

The system is specifically optimized for Stereolabs' ZED camera setups and provides:

- **Real-time performance**: Optimized for real-time robotic applications
- **Robust tracking**: Advanced keyframe selection and pose optimization
- **ROS 2 native**: Built from the ground up for the ROS 2 ecosystem
- **Cross-platform compatibility**: Designed to run on both x86 and ARM architecture machines
- **Modular design**: Clean, extensible architecture for research and development
- **Developer-friendly**: Easy-to-read code accessible even to non-SLAM experts

### Key Components

- **Visual Frontend**: Stereo camera processing and feature extraction
- **Keyframe Management**: Intelligent keyframe selection for mapping
- **Pose Optimization**: Bundle adjustment using g2o
- **Transform Integration**: Seamless integration with ROS 2 tf2 system

## Features

✅ **Real-time stereo visual SLAM**  
✅ **ROS 2 Humble compatibility out of the box**  
✅ **Independent of `zed-ros2-wrapper`**  
✅ **Leverages ZED SDK and CUDA - popular, mature industry-standard libraries**  
✅ **Cross-platform support for x86 and ARM machines - easy deployment on NVIDIA development kits**  
✅ **Minimal dependencies for simplified maintenance and code clarity**  
✅ **Advanced graph-based pose optimization with g2o**  
✅ **Lie algebra-based transformations with Sophus**  
✅ **Modular and extensible architecture**  
✅ **TF2 integration for coordinate transformations - seamless robot integration**

### 🚧 **Planned Features**
🔄 Enhanced cross-platform compatibility  
🔄 Modern loop closure detection  
🔄 Generalized visual odometry frontend (support for additional camera types)  
🔄 IMU integration  
🔄 Dense mapping capabilities  
🔄 Plugin-based architecture  
🔄 Benchmark tools  
🔄 Real-time tuning and visualization  
🔄 Comprehensive automated CI/CD pipeline  
🔄 Containerization support  

## Dependencies

Orbis SLAM requires several key dependencies for optimal performance:

### Core Libraries

| Library | Version | Purpose | Installation |
|---------|---------|---------|--------------|
| **Ceres Solver** | Latest | Non-linear optimization | Build from source (included as submodule) |
| **Sophus** | Latest | Lie algebra operations | Build from source (included as submodule) |
| **g2o** | Latest | Graph optimization | Build from source (included as submodule) |
| **Eigen3** | 3.3+ | Linear algebra | `sudo apt install libeigen3-dev` |
| **TBB** | Latest | Parallel computing | `sudo apt install libtbb-dev` |
| **FMT** | Latest | String formatting | `sudo apt install libfmt-dev` |

### ROS 2 Dependencies

| Package | Purpose |
|---------|---------|
| `rclcpp` | ROS 2 C++ client library |
| `sensor_msgs` | Sensor message types |
| `geometry_msgs` | Geometry message types |
| `nav_msgs` | Navigation message types |
| `cv_bridge` | OpenCV-ROS bridge |
| `image_transport` | Image transport |
| `tf2_ros` | Transform library |
| `tf2_geometry_msgs` | TF2 geometry utilities |

### Dataset recording and playback

| Library | Version | Purpose | Installation |
|---------|---------|---------|--------------|
| **protobuf** | 3 | Data structure and engine to record and playback custom datasets used to evaluate this project | |

To buoild the `.proto ` file, please insert the following commands:

```bash
# Change directory to the "proto" directory under this project's root directory
cd /path/to/this/project/root/dir/proto

# Build the proto file to be used by CPP programs
protoc -I=. --cpp_out=. zed_recording.proto
```

### Hardware Dependencies

- **x86 or ARM machine**: A laptop or development kit with CUDA-enabled GPU
- **ZED Camera**: Stereo camera (ZED, ZED 2, ZED 2i, ZED X)
- **CUDA-capable GPU**: Strongly recommended for optimal performance
- **ZED SDK**: Latest version from Stereolabs

### System Requirements

- **OS**: Ubuntu 20.04+ or Ubuntu 22.04 (recommended)
- **ROS 2**: Humble Hawksbill
- **Compiler**: GCC 9+ or Clang 10+
- **CMake**: 3.24+
- **Memory**: 8GB+ RAM recommended
- **Storage**: 2GB+ free space

## Installation

Follow these steps to install Orbis SLAM from source on your system.

### Prerequisites

First, ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
```

### Step 1: Install System Dependencies

#### CUDA

Please refer to the [CUDA installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) to set up your CUDA environment.

#### Eigen3 (Linear Algebra Library)
```bash
sudo apt install -y libeigen3-dev libtf2-eigen-dev
```

#### FMT (String Formatting Library)
```bash
sudo apt install -y libfmt-dev
```

#### Intel TBB (Threading Building Blocks)
```bash
sudo apt install -y libtbb-dev
```

### Step 2: Build Third-Party Libraries

Orbis SLAM includes several third-party libraries as submodules. Initialize and build them:

```bash
# Clone the repository if you haven't already
git clone https://github.com/Megacephalo/Orbis_SLAM.git
cd Orbis_SLAM

# Initialize submodules
git submodule update --init --recursive
```

#### Build Ceres Solver

Ceres is essential for non-linear optimization and is a dependency of the Sophus library. Build it from the included submodule:

```bash
cd third_party/ceres-solver
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build -j$(nproc)
cd build && sudo make install
cd ../../..
```

#### Build Sophus

Sophus provides Lie algebra operations for pose transformations:

```bash
cd third_party/Sophus
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build -j$(nproc)
cd build && sudo make install
cd ../../..
```

> **Note for Limited Memory Systems**: If building fails due to insufficient memory (e.g., on NVIDIA Jetson devices), use single-threaded compilation:
> ```bash
> cmake --build build -j1
> ```

### Step 3: ZED SDK Installation

#### Clean Previous Installation (if applicable)

If you have a previous ZED SDK installation, remove it first:

```bash
# Remove SDK files
sudo rm -rf /usr/local/zed

# Uninstall Python wrapper
pip3 uninstall pyzed

# Remove configuration data
rm -rf ~/.config/zed ~/.local/share/zed
```

#### Install ZED SDK

1. Download the ZED SDK from [Stereolabs Download Page](https://www.stereolabs.com/developers/release/)
2. Install the SDK:

```bash
# Install zstd if needed
sudo apt install -y zstd

# Make installer executable
chmod +x ZED_SDK_Ubuntu*.zstd.run

# Run the installer
./ZED_SDK_Ubuntu*.zstd.run
```

3. Follow the installation prompts
4. Reboot your system:

```bash
sudo reboot
```

### Step 4: Build Orbis SLAM

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

### Verification

Verify the installation by checking if the node can be launched:

```bash
ros2 run orbis_slam orbis_slam_main
```

If successful, you should see the Orbis SLAM node initialize with messages related to the camera boot status.

## Quick Start

Get up and running with Orbis SLAM in just a few commands:

### 1. Connect Your ZED Camera

Ensure your ZED camera is properly connected via USB 3.0 or USB-C. **Tip**: Test the camera first with ZED SDK sample code or the **zed-ros2-wrapper** launch files to ensure proper functionality.

### 2. Launch the SLAM System

```bash
# Source your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch Orbis SLAM with sensor-only mode (treats the camera as its own robot)
ros2 launch orbis_slam orbis_slam_sensor_only.py

# Or run the main node directly. See "ROS 2 Topics" section for configurable topics and parameters
ros2 run orbis_slam orbis_slam_main
```

### 3. Verify Operation

Check that the system is publishing transforms and processing frames:

```bash
# GUI to visualize TF tree and node graph
rqt

# Monitor TF tree
ros2 run tf2_tools view_frames

# Check published topics
ros2 topic list

# Monitor pose output
ros2 topic echo /orbis_slam/pose
```

## Usage

### Basic Operation

Orbis SLAM operates as a ROS 2 node that processes stereo camera data and publishes pose estimates and transforms. The system automatically:

1. **Initializes** the ZED camera and calibration
2. **Processes** incoming stereo frames
3. **Selects** keyframes for mapping
4. **Optimizes** poses using bundle adjustment
5. **Publishes** transforms to the ROS 2 TF tree

### Launch Files

#### Sensor-Only Mode
```bash
ros2 launch orbis_slam orbis_slam_sensor_only.py
```

This launch file starts Orbis SLAM in sensor-only mode to facilitate testing with only the camera. This launch file also serves as an example to demonstrate the ROS 2 node signature and how to integrate it with your mobile platform.

### ROS 2 Topics

#### Subscribed Topics
- `/zed/zed_node/left/image_rect_color` - Left camera image
- `/zed/zed_node/right/image_rect_color` - Right camera image
- `/zed/zed_node/depth/depth_registered` - Depth image

#### Published Topics
- `/orbis_slam/pose` - Current pose estimate
- `/orbis_slam/trajectory` - Complete trajectory
- `/tf` - Transform tree updates

### Parameters

Key parameters can be configured through ROS 2 parameters:

```yaml
orbis_slam:
  ros__parameters:
    world_frame: "map"
    odom_frame: "odom"
    robot_baselink_frame: "base_link"
    left_camera_frame: "zed_left_camera_frame"
    enable_slam: true
```

- **enable_slam**: When enabled, the node runs global pose optimization on selected keyframes to optimize the camera's pose in the world frame. When disabled, the camera's global pose matches the pose estimated from odometry.

## Configuration

### Camera Calibration

Orbis SLAM automatically uses the calibration parameters from your ZED camera. For custom stereo setups, modify the camera parameters in the configuration files.

### Frame Configuration

The system uses standard ROS 2 frame conventions:

- `map` - Global reference frame
- `odom` - Odometry frame
- `base_link` - Robot base frame
- `zed_left_camera_frame` - Left camera frame

### Performance Tuning

For optimal performance:

1. **GPU Memory**: Ensure sufficient GPU memory (4GB+ recommended)
2. **CPU Cores**: Multi-core systems perform better with parallel optimization
3. **Camera FPS**: Higher frame rates improve tracking accuracy but increase computational load

## Architecture

### System Overview

```
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│   ZED Camera    │────│  Visual Frontend │────│ Keyframe Selector│
└─────────────────┘    └──────────────────┘    └──────────────────┘
                                │                        │
                                ▼                        ▼
┌─────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  TF2 Publisher  │◄───│ Pose Optimization│◄───│   Map Manager    │
└─────────────────┘    └──────────────────┘    └──────────────────┘
```

### Core Components

#### 1. **OrbisSLAMPipeline** (`orbis_slam_pipeline.h`)
- Main processing pipeline
- Coordinates all subsystems
- Manages ROS 2 node lifecycle

#### 2. **ZEDWrapper** (`zed_wrapper.h`)
- Interfaces with ZED SDK
- Handles camera initialization and frame capture
- Provides calibration data

#### 3. **KeyFrameSelector** (`keyframe_selector.h`)
- Implements keyframe selection strategy
- Manages frame-to-frame tracking
- Decides when to create new keyframes

#### 4. **PoseOptimizer** (`pose_optimizer.h`)
- Performs bundle adjustment using Ceres Solver
- Implements pose graph optimization with g2o
- Handles local and global optimization

#### 5. **Essential Data Structures** (`essential_data_structure.h`)
- Defines core data types (Frame, Trajectory, etc.)
- Provides thread-safe, efficient pose storage and access (i.e., the trajectory)

## Roadmap

### Current Features (v0.1.0)
- ✅ Real-time stereo visual SLAM
- ✅ ZED camera integration
- ✅ IMU integration for VIO
- ✅ ROS 2 Humble support
- ✅ TF2 integration
- ✅ Pose optimization with g2o

### Upcoming Features (v0.2.0)
- 🔄 Modern loop closure detection
- 🔄 Dense mapping capabilities
- 🔄 Real-time visualization tools
- 🔄 Smooth installation and setup experience

### Future Development
- 📋 Performance benchmarking suite
- 📋 Docker containerization
- 📋 Integration with Nav2 stack

### Performance Goals
- Real-time operation at 30 FPS on modern hardware
- Memory usage under 2GB for typical indoor environments
- Accuracy within 1% of trajectory length for standard datasets and live streaming

## Contributing

We welcome contributions to Orbis SLAM! Here's how you can help:

### Getting Started

1. **Fork** the repository
2. **Clone** your fork locally
3. **Create** a feature branch
4. **Make** your changes
5. **Test** thoroughly
6. **Submit** a pull request

### Development Guidelines

- Follow the existing code style and conventions
- Add unit tests for new functionality
- Update documentation for API changes
- Ensure all tests pass before submitting

### Code Style

- Use modern C++17 features
- Follow Google C++ Style Guide
- Use meaningful variable and function names
- Add comprehensive comments for complex algorithms

### Reporting Issues

Please use the [GitHub Issues](https://github.com/Megacephalo/Orbis_SLAM/issues) page to report bugs or request features. Include:

- Detailed description of the issue
- Steps to reproduce
- System specifications
- ROS 2 and camera setup details

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Third-Party Libraries

- **Ceres Solver**: [New BSD License](https://ceres-solver.org/license.html)
- **Sophus**: [MIT License](https://github.com/strasdat/Sophus/blob/master/LICENSE.txt)
- **g2o**: [BSD License](https://github.com/RainerKuemmerle/g2o/blob/master/LICENSE)
- **ZED SDK**: [Stereolabs License](https://www.stereolabs.com/legal/license/)

---

<div align="center">

**Built with ❤️ for the robotics community**

[Report Bug](https://github.com/Megacephalo/Orbis_SLAM/issues) · [Request Feature](https://github.com/Megacephalo/Orbis_SLAM/issues) · [Contribute](https://github.com/Megacephalo/Orbis_SLAM/pulls)

</div>

