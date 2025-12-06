# Enable AUTOMOC for Qt Meta-Object Compiler
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create library with source files
# Note: PROTO_SRCS and PROTO_HDRS are generated in parent scope
# Use SHARED library for proper Qt MOC integration
add_library(Orbis_SLAM_pipeline SHARED
    orbis_slam_pipeline.cpp
    pose_optimizer.cpp
    keyframe_selector.cpp
    zed_recorder.cpp
    zed_backend_worker.cpp
    ${PROJECT_SOURCE_DIR}/include/orbis_slam/recorder_and_player/zed_backend_worker.h
)

# Add generated protobuf sources if they exist
# if(PROTO_SRCS AND PROTO_HDRS)
#     target_sources(Orbis_SLAM_pipeline PRIVATE ${PROTO_SRCS} ${PROTO_HDRS})
# endif()
target_include_directories(Orbis_SLAM_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
set_target_properties(Orbis_SLAM_pipeline PROPERTIES
    AUTOMOC ON
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
    Qt6::Core
    # protobuf::libprotobuf  # Link protobuf library
)

# Install the shared library
install(TARGETS Orbis_SLAM_pipeline
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)