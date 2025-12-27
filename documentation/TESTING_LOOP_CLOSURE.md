# Testing Loop Closure Detection

This guide explains how to test the newly implemented loop closure detection system in Orbis SLAM.

## Prerequisites

1. **DBoW3 Library**: Already installed as a system library
2. **Vocabulary File**: Located at `third_party/DBow3/orbvoc.dbow3`
3. **Built Package**: Run `colcon build --packages-select orbis_slam`

## Quick Start

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Run with Loop Closure Enabled

**Option A: Using the launch file**

```bash
ros2 launch orbis_slam orbis_slam_with_loop_closure.py
```

**Option B: Direct node execution with parameters**

```bash
ros2 run orbis_slam orbis_slam_node \
  --ros-args \
  -p enable_slam:=true \
  -p enable_loop_closure:=true \
  -p vocabulary_path:=$HOME/ros2_ws/src/orbis_slam/third_party/DBow3/orbvoc.dbow3
```

**Option C: Custom vocabulary path**

```bash
ros2 launch orbis_slam orbis_slam_with_loop_closure.py \
  vocabulary_path:=/path/to/your/vocabulary.dbow3
```

## Test Scenarios

### Scenario 1: Simple Loop Closure Test

**Objective**: Verify basic loop closure detection in a controlled environment.

**Steps**:
1. Start the system with loop closure enabled
2. Move the robot/camera in a rectangular path
3. Return to the starting position
4. Observe console output for loop detection messages

**Expected Output**:
```
[LoopClosureDetector] Loop detected! Query KF: 45 -> Match KF: 3 (score: 0.812)
Loop closure callback: KF 45 -> KF 3 (score: 0.812)
Global optimization triggered by loop closure.
```

### Scenario 2: Multiple Loop Closures

**Objective**: Test system with multiple revisited locations.

**Steps**:
1. Create a figure-8 trajectory
2. Cross the center point multiple times
3. Monitor for multiple loop detections

**Expected Behavior**:
- Loop closures detected at each crossing
- Pose graph updated with new constraints
- Global optimization smooths trajectory

### Scenario 3: Parameter Tuning

**Objective**: Find optimal parameters for your environment.

**Test Different Similarity Thresholds**:

Edit [loop_closure_detector.cpp:37-42](src/loop_closure_detector.cpp#L37-L42) and rebuild:

```cpp
loop_closure_detector_ = LoopClosureDetector::create(
    vocabulary_path_,
    covisibility_graph_,
    trajectory_,
    0.65,  // Try: 0.65, 0.7, 0.75, 0.8
    30     // Try: 20, 30, 40, 50
);
```

**Similarity Threshold Guide**:
- `0.6-0.65`: More loop closures, higher false positive rate
- `0.7-0.75`: Balanced (recommended)
- `0.8-0.85`: Fewer loop closures, very conservative

**Temporal Separation Guide**:
- `20-25`: Detects loops sooner (good for small environments)
- `30-40`: Balanced (recommended)
- `50+`: Very conservative (good for large environments)

## Monitoring and Debugging

### 1. Console Output

**Normal Operation**:
```
[OrbisSLAMPipeline] Initializing loop closure detection...
[LoopClosureDetector] Loading vocabulary from: /path/to/orbvoc.dbow3
[LoopClosureDetector] Vocabulary loaded successfully.
[OrbisSLAMPipeline] Loop closure detection started successfully.
```

**Loop Detection**:
```
[LoopClosureDetector] Loop detected! Query KF: X -> Match KF: Y (score: Z)
Loop closure callback: KF X -> KF Y (score: Z)
Global optimization triggered by loop closure.
```

**Feature Extraction** (debug level):
```
Extracted 987 features for frame 42 (756 with valid 3D points)
```

### 2. Check System Status

**Verify vocabulary file exists**:
```bash
ls -lh ~/ros2_ws/src/orbis_slam/third_party/DBow3/orbvoc.dbow3
# Should show ~47MB file
```

**Monitor ROS parameters**:
```bash
ros2 param list /orbis_slam_node
ros2 param get /orbis_slam_node enable_loop_closure
ros2 param get /orbis_slam_node vocabulary_path
```

### 3. Performance Metrics

**Check CPU usage**:
```bash
top -p $(pgrep -f orbis_slam_node)
```

**Expected CPU usage**:
- Main thread: 20-40% (with feature extraction)
- Loop closure thread: 5-15% (varies with keyframe rate)

**Memory usage**:
- Base: ~200MB
- +DBoW3 vocabulary: ~50MB
- +Per 100 keyframes: ~3-5MB

## Troubleshooting

### Issue 1: Vocabulary Not Loading

**Symptom**:
```
[LoopClosureDetector] Failed to load vocabulary: ...
Loop closure enabled but vocabulary_path not set. Disabling loop closure.
```

**Solutions**:
1. Check file path: `ls ~/ros2_ws/src/orbis_slam/third_party/DBow3/orbvoc.dbow3`
2. Use absolute path in launch file
3. Verify file permissions: `chmod 644 orbvoc.dbow3`

### Issue 2: No Loop Closures Detected

**Symptom**: No "Loop detected!" messages after revisiting locations.

**Possible Causes**:
1. **Similarity threshold too high**: Lower it to 0.65
2. **Insufficient features**: Check debug output for feature count
3. **Temporal separation too large**: Reduce to 20-25
4. **Poor lighting/texture**: ORB features need textured environment

**Debug Steps**:
```bash
# Enable debug logging (rebuild after editing)
# In orbis_slam_pipeline.cpp, change RCLCPP_DEBUG to RCLCPP_INFO
```

### Issue 3: False Loop Closures

**Symptom**: Loop closures detected in non-overlapping areas.

**Solutions**:
1. Increase similarity threshold to 0.75-0.8
2. Increase min temporal separation to 40-50
3. Check geometric verification parameters in [loop_closure_detector.cpp:213-229](src/loop_closure_detector.cpp#L213-L229)

### Issue 4: Build Errors

**DBoW3 not found**:
```bash
# Verify DBoW3 installation
find /usr -name "*DBoW3*"
pkg-config --modversion DBoW3
```

If not installed as system library:
```bash
cd ~/ros2_ws/src/orbis_slam/third_party/DBow3
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

## Performance Testing

### Test 1: Feature Extraction Speed

**Record timing**:
Add timing code in [orbis_slam_pipeline.cpp:295-346](src/orbis_slam_pipeline.cpp#L295-L346):

```cpp
auto start = std::chrono::high_resolution_clock::now();
feature_extractor_->extract(image, keypoints, descriptors);
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
RCLCPP_INFO(this->get_logger(), "Feature extraction: %ld ms", duration.count());
```

**Expected**: 10-25ms per frame

### Test 2: Loop Detection Speed

Monitor console output for processing time:
- DBoW3 query: 5-10ms
- Geometric verification: 20-50ms
- Total per keyframe: 30-80ms

### Test 3: Memory Growth

```bash
# Monitor memory over time
watch -n 1 "ps aux | grep orbis_slam_node | grep -v grep | awk '{print \$4, \$5, \$6}'"
```

**Expected growth**: ~3-5MB per 100 keyframes (linear)

## Advanced Configuration

### Custom Feature Parameters

Edit [orbis_slam_pipeline.cpp:31](src/orbis_slam_pipeline.cpp#L31):

```cpp
feature_extractor_ = FeatureExtractor::create(
    1000,  // num_features: try 500, 750, 1000, 1500
    1.2f,  // scale_factor: try 1.1, 1.2, 1.5
    8      // num_levels: try 6, 8, 10
);
```

### Custom Covisibility Threshold

Edit [orbis_slam_pipeline.cpp:34](src/orbis_slam_pipeline.cpp#L34):

```cpp
covisibility_graph_ = CovisibilityGraph::create(
    15  // min shared points: try 10, 15, 20, 30
);
```

### Geometric Verification Tuning

Edit [loop_closure_detector.cpp:213-229](src/loop_closure_detector.cpp#L213-L229):

```cpp
if (matches.size() < 20) {  // Try: 15, 20, 30
    return false;
}

size_t num_good_matches = static_cast<size_t>(matches.size() * 0.7);  // Try: 0.6, 0.7, 0.8

if (points_query.size() < 15) {  // Try: 10, 15, 20
    return false;
}
```

## Validation

### Visual Verification

1. **Plot trajectory**: Use RViz to visualize `/odom` path
2. **Before loop closure**: Trajectory will drift
3. **After loop closure**: Trajectory should close the loop

### Quantitative Metrics

**Pose graph statistics**:
```bash
# Check number of loop closure constraints
grep "Loop closure callback" /path/to/log | wc -l
```

**Optimization frequency**:
```bash
# Check global optimization triggers
grep "Global optimization triggered" /path/to/log | wc -l
```

## Example Test Results

### Successful Loop Closure

```
Keyframe 0-29: Building initial map
Keyframe 30-59: Exploring new area
Keyframe 60: Loop detected! (score: 0.782)
  - Match: Keyframe 5
  - Shared features: 127
  - Geometric verification: PASSED
  - Loop constraint added to pose graph
  - Global optimization: 156ms
  - Trajectory corrected by 0.45m translation, 3.2° rotation
```

### Parameter Comparison

| Similarity | Temporal Sep | Loops Detected | False Positives | CPU Usage |
|------------|--------------|----------------|-----------------|-----------|
| 0.65       | 20           | 12             | 2               | 25%       |
| 0.70       | 30           | 8              | 0               | 22%       |
| 0.75       | 40           | 5              | 0               | 20%       |
| 0.80       | 50           | 3              | 0               | 18%       |

**Recommended**: similarity=0.70, temporal_sep=30

## Next Steps

After successful testing:

1. **Tune parameters** for your specific environment
2. **Record datasets** for repeatability testing
3. **Evaluate accuracy** using ground truth if available
4. **Optimize performance** if needed (reduce features, etc.)
5. **Deploy** to your robot platform

## Support

For issues or questions:
1. Check build output: `colcon build --packages-select orbis_slam`
2. Review log files in `~/.ros/log/`
3. Enable verbose logging with RCLCPP_DEBUG
4. Check [LOOP_CLOSURE_IMPLEMENTATION.md](LOOP_CLOSURE_IMPLEMENTATION.md) for architecture details
