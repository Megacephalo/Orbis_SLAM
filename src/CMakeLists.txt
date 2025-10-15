add_library(Orbis_SLAM_pipeline
    orbis_slam_pipeline.cpp
    pose_optimizer.cpp
    keyframe_selector.cpp
)
target_include_directories(Orbis_SLAM_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(Orbis_SLAM_pipeline
    rclcpp
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    geometry_msgs
)
# Require C99 and C++17
target_compile_features(Orbis_SLAM_pipeline PUBLIC 
    c_std_99 
    cxx_std_17
)
target_link_libraries(Orbis_SLAM_pipeline
    ${ZED_LIBS}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    Sophus::Sophus
    fmt::fmt
    g2o::core
    g2o::types_slam3d
    g2o::solver_eigen
    g2o::stuff
    TBB::tbb
)