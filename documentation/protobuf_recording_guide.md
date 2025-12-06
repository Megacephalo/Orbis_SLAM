# Protobuf Recording System for ZED Camera SLAM Data

## Overview

This document describes the Protocol Buffers (Protobuf) based recording system for capturing ZED stereo camera data for SLAM pipeline processing and playback.

## Schema Design

The recording format captures all essential data needed to:
1. Replay SLAM pipeline processing offline
2. Reconstruct 3D point clouds from depth images
3. Evaluate and debug SLAM performance
4. Train machine learning models

### File Format

**Extension**: `.pb` or `.orbis` (Orbis SLAM Recording)

**Structure**: Single binary file containing:
- Camera calibration (intrinsics + stereo calibration)
- Sequential frames with:
  - Timestamps and indices
  - Compressed stereo images (left/right)
  - Registered depth images
  - Camera poses (6-DOF SE3)
  - Keyframe flags and quality metrics

## Message Definitions

### Core Data Types

#### 1. `CameraIntrinsics`
Camera intrinsic parameters for image rectification and 3D reconstruction.

```protobuf
message CameraIntrinsics {
    double fx, fy;           // Focal lengths (pixels)
    double cx, cy;           // Principal point (pixels)
    repeated double distortion;  // [k1, k2, p1, p2, k3, ...]
    uint32 width, height;    // Image dimensions
}
```

**Usage**: Stored once per recording for left and right cameras.

#### 2. `StereoCalibration`
Extrinsic stereo calibration between left and right cameras.

```protobuf
message StereoCalibration {
    double baseline;              // Meters between cameras
    repeated double rotation;     // 3x3 rotation matrix (row-major)
    repeated double translation;  // 3D translation vector [tx, ty, tz]
}
```

**Usage**: Constant for the recording session.

#### 3. `CameraPose`
6-DOF camera pose in SE(3) group.

```protobuf
message CameraPose {
    double tx, ty, tz;          // Translation (meters)
    double qw, qx, qy, qz;      // Quaternion rotation
    repeated double rotation_matrix;  // Optional 3x3 matrix
}
```

**Convention**:
- Pose represents **world-to-camera** transform
- Quaternion format: `[w, x, y, z]` (Hamilton convention)
- Right-handed coordinate system

#### 4. `CompressedImage`
Image data with flexible compression options.

```protobuf
message CompressedImage {
    enum Format {
        PNG  = 1;    // Lossless, best for depth
        JPEG = 2;    // Lossy, good for RGB
        WEBP = 3;    // Balanced compression
        RAW  = 4;    // Uncompressed
    }
    Format format;
    bytes data;              // Compressed/raw image bytes
    uint32 width, height;
    uint32 channels;         // 1=gray, 3=RGB, 4=RGBA
}
```

**Recommendations**:
- **RGB Images**: JPEG (quality 90-95) or WEBP for file size
- **Depth Images**: PNG (16-bit) for lossless depth preservation
- **Grayscale**: PNG for lossless or JPEG for smaller files

#### 5. `Frame`
Complete data for a single timestep.

```protobuf
message Frame {
    uint64 frame_index;      // Unique sequential ID
    uint64 timestamp_ns;     // Nanoseconds since epoch

    CompressedImage left_image;
    CompressedImage right_image;
    CompressedImage depth_image;  // Registered to left camera

    CameraPose pose;         // Camera pose at this frame
    bool is_keyframe;        // SLAM keyframe flag
    double confidence;       // Pose quality [0-1]
}
```

#### 6. `Recording`
Top-level message containing full session.

```protobuf
message Recording {
    string recording_id;
    uint64 start_timestamp_ns;
    uint64 end_timestamp_ns;

    CameraIntrinsics left_intrinsics;
    CameraIntrinsics right_intrinsics;
    StereoCalibration stereo_calibration;

    repeated Frame frames;   // All recorded frames

    double fps;
    string camera_model;     // "ZED2", "ZED2i", etc.
    string description;
}
```

## Recording Strategies

### Strategy 1: Complete Recording (Batch)
Store all frames in memory, serialize once at the end.

**Pros**:
- Simple implementation
- Single file write
- Easy metadata updates

**Cons**:
- High memory usage
- Data loss if crash before save
- Not suitable for long recordings

**Use Case**: Short calibration sequences, testing

### Strategy 2: Incremental Recording (Streaming)
Write frames incrementally to disk.

**Pros**:
- Constant memory usage
- Progressive data saving
- Safe for long recordings

**Cons**:
- Requires careful file handling
- Need to update header/metadata

**Use Case**: Long SLAM sessions, production use

**Implementation Approach**:
```
1. Write placeholder Recording header (with empty frames)
2. For each frame:
   - Serialize Frame message
   - Write delimited to file (length-prefixed)
3. On completion:
   - Seek to start
   - Update header with metadata
```

### Strategy 3: Frame-by-Frame Files
Save each frame as separate `.pb` file.

**Pros**:
- Simplest implementation
- Parallel processing friendly
- Selective frame loading

**Cons**:
- Many files to manage
- Overhead per file
- Calibration data duplication

**Use Case**: Distributed processing, frame-level datasets

## File Size Estimation

For 1920x1080 stereo + depth at 30 FPS:

| Component | Size/Frame | 10s | 60s |
|-----------|------------|-----|-----|
| Left RGB (JPEG 90%) | ~100 KB | 30 MB | 180 MB |
| Right RGB (JPEG 90%) | ~100 KB | 30 MB | 180 MB |
| Depth (PNG 16-bit) | ~200 KB | 60 MB | 360 MB |
| Pose + Metadata | ~200 B | 60 KB | 360 KB |
| **Total** | **~400 KB** | **120 MB** | **720 MB** |

**Notes**:
- JPEG quality 90%: good balance of size/quality
- Depth compression ratio ~2-3x with PNG
- Can use WEBP for 20-30% smaller RGB images

## Point Cloud Reconstruction

Given a frame, reconstruct 3D points:

```python
def reconstruct_point_cloud(frame, left_intrinsics):
    # Decompress depth image
    depth = decode_image(frame.depth_image)  # HxW depth map

    # Camera intrinsics
    fx, fy = left_intrinsics.fx, left_intrinsics.fy
    cx, cy = left_intrinsics.cx, left_intrinsics.cy

    # Generate point cloud
    points_3d = []
    for v in range(height):
        for u in range(width):
            z = depth[v, u]
            if z > 0:  # Valid depth
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points_3d.append([x, y, z])

    # Transform to world frame
    pose = frame.pose
    T_world_camera = pose_to_transform_matrix(pose)
    points_world = transform_points(points_3d, T_world_camera)

    return points_world
```

## Reading Recordings

### C++ Example
```cpp
#include "zed_recording.pb.h"
#include <fstream>

// Read complete recording
orbis_slam::Recording recording;
std::ifstream input("recording.pb", std::ios::binary);
recording.ParseFromIstream(&input);

// Access frame data
for (const auto& frame : recording.frames()) {
    std::cout << "Frame " << frame.frame_index()
              << " at " << frame.timestamp_ns() << "ns\n";

    // Decompress images
    cv::Mat left = decode_image(frame.left_image());
    cv::Mat depth = decode_image(frame.depth_image());

    // Extract pose
    Sophus::SE3d pose = extract_pose(frame.pose());
}
```

### Python Example
```python
import zed_recording_pb2

# Read recording
recording = zed_recording_pb2.Recording()
with open("recording.pb", "rb") as f:
    recording.ParseFromString(f.read())

# Iterate frames
for frame in recording.frames:
    print(f"Frame {frame.frame_index} at {frame.timestamp_ns}ns")

    # Decompress images
    left_img = decode_image(frame.left_image)
    depth_img = decode_image(frame.depth_image)

    # Get pose
    pose = np.array([
        frame.pose.tx, frame.pose.ty, frame.pose.tz,
        frame.pose.qw, frame.pose.qx, frame.pose.qy, frame.pose.qz
    ])
```

## Best Practices

### 1. Image Compression
- **RGB**: JPEG 90% or WEBP 85% quality
- **Depth**: PNG 16-bit (lossless)
- **Grayscale tracking**: JPEG 85% sufficient

### 2. Coordinate Systems
- **ZED Camera Frame**: Right-handed, +X right, +Y down, +Z forward
- **World Frame**: Consistent with your SLAM convention
- Document frame conventions in `description` field

### 3. Timestamp Synchronization
- Use ZED camera timestamps (`zed.getTimestamp()`)
- Store in nanoseconds for precision
- Ensure monotonic increasing sequence

### 4. Depth Image Format
- Store as 16-bit or 32-bit float
- Units: millimeters (uint16) or meters (float32)
- Document units in `description` field

### 5. Keyframe Selection
- Mark frames used in SLAM optimization
- Enables efficient replay without full processing
- Typical: 1 keyframe per 0.5-1.0 meters traveled

## Integration with ZED SDK

### Extracting ZED Data
```cpp
// Get camera parameters
auto calib = zed.getCameraInformation().camera_configuration;
auto left_calib = calib.calibration_parameters.left_cam;
auto right_calib = calib.calibration_parameters.right_cam;

// Create CameraIntrinsics message
orbis_slam::CameraIntrinsics* left_intrinsics = recording.mutable_left_intrinsics();
left_intrinsics->set_fx(left_calib.fx);
left_intrinsics->set_fy(left_calib.fy);
left_intrinsics->set_cx(left_calib.cx);
left_intrinsics->set_cy(left_calib.cy);
left_intrinsics->set_width(left_calib.image_size.width);
left_intrinsics->set_height(left_calib.image_size.height);

// Distortion coefficients
for (double d : left_calib.disto) {
    left_intrinsics->add_distortion(d);
}

// Stereo baseline
orbis_slam::StereoCalibration* stereo = recording.mutable_stereo_calibration();
stereo->set_baseline(calib.calibration_parameters.getCameraBaseline());
```

## Tools and Utilities

### Provided Tools (to be implemented):
1. **`orbis_recording_info`**: Display recording metadata
2. **`orbis_extract_frames`**: Export frames as images
3. **`orbis_replay`**: Replay recording through SLAM pipeline
4. **`orbis_convert`**: Convert between formats (Protobuf, ROS bag, etc.)
5. **`orbis_validate`**: Check recording integrity

## Future Extensions

1. **IMU Data**: Add accelerometer/gyroscope for VIO
2. **Loop Closure**: Store global features for loop detection
3. **Semantic Labels**: Per-pixel semantic segmentation
4. **Event Logging**: System events, errors, state changes
5. **Multi-Camera**: Support for more than stereo pairs

## References

- Protocol Buffers: https://protobuf.dev/
- ZED SDK Documentation: https://www.stereolabs.com/docs/
- Sophus Library (SE3): https://github.com/strasdat/Sophus