# Orbis SLAM Code Reference

This Doxygen site documents the public headers, implementation files, ROS 2 launch scripts, protobuf schema, and project design notes for Orbis SLAM.

Start with these areas:

- `Orbis::OrbisSLAMPipeline` for the core SLAM processing pipeline.
- `Orbis::Frame` and `Orbis::Trajectory` for the primary frame and trajectory data model.
- `Orbis::KeyFrameSelector`, `Orbis::FeatureExtractor`, `Orbis::CovisibilityGraph`, `Orbis::LoopClosureDetector`, and `Orbis::PoseOptimizer` for the SLAM subsystems.
- The Markdown documentation pages for loop closure, protobuf recording/playback, testing, and technical notes.

To regenerate this documentation from the repository root:

```bash
doxygen documentation/Doxyfile
```

Open `documentation/doxygen/html/index.html` in a browser to view the generated documentation.
