# AGENT.md

This repo is a ROS 2 package for Orbis SLAM. Treat it as an active refactor, not a finished release.

## Where things live

- `include/orbis_slam/`: public headers for the SLAM core and support types
- `src/`: core library implementation
- `app/`: ROS wrapper executables and playback/live entry points
- `launch/`: ROS 2 launch files
- `proto/`: protobuf schema and generated recording code
- `third_party/`: vendored dependencies and submodules

## Current shape of the code

- `OrbisSLAMPipeline` is now a standalone C++ library class, not a ROS node.
- ROS lifecycle, TF broadcasting, image publication, and playback wiring live in `app/`.
- `orbis_slam_node` currently maps to the live wrapper path in `app/zed_live_node.cpp`.
- `orbis_slam_playback_node` processes protobuf recordings through the same SLAM pipeline.

## Build and run

- Build from the workspace root with:
  `colcon build --packages-select orbis_slam --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Source the workspace with:
  `source install/setup.bash`
- Common executables:
  - `ros2 run orbis_slam orbis_slam_node`
  - `ros2 run orbis_slam orbis_slam_playback_node`
  - `ros2 run orbis_slam ZEDRecPlay`

## Things to watch

- The worktree may already contain user changes. Do not revert unrelated edits.
- `app/CMakeLists.txt` and `app/zed_live_node.cpp` have been edited locally in this branch.
- Some third-party dependencies are vendored as submodules. If a build fails around `Sophus`, `ceres-solver`, or `g2o`, verify the submodule checkout before changing code.
- Keep changes narrow and aligned with the existing refactor direction: pipeline logic in `src/`, ROS integration in `app/`.

## Practical default

- Read `CLAUDE.md` first for build and architecture context.
- Prefer the existing CMake and launch patterns instead of introducing new wrappers or parallel entry points.
