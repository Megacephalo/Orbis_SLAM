# Enable automatic MOC, UIC, and RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt6 COMPONENTS Core Gui OpenGL Widgets OpenGLWidgets REQUIRED)
find_package(OpenGL REQUIRED)

find_package(glm REQUIRED)

# Set OpenGL preference policy
set(OpenGL_GL_PREFERENCE GLVND)
cmake_policy(SET CMP0072 NEW)

# orbis_slam_node
add_executable(${PROJECT_NAME}_node
    orbis_slam_main.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
## Require C99 and C++17
target_compile_features(${PROJECT_NAME}_node  PUBLIC 
    c_std_99 
    cxx_std_17
)
target_link_libraries(${PROJECT_NAME}_node
    Orbis_SLAM_pipeline
)

# orbis_slam_playback_node
add_executable(${PROJECT_NAME}_playback_node
    zed_playback_node.cpp
)
target_include_directories(${PROJECT_NAME}_playback_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_BINARY_DIR}/proto  # For generated protobuf headers
)
## Require C99 and C++17
target_compile_features(${PROJECT_NAME}_playback_node PUBLIC
    c_std_99
    cxx_std_17
)
ament_target_dependencies(${PROJECT_NAME}_playback_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
)
target_link_libraries(${PROJECT_NAME}_playback_node
    Orbis_SLAM_pipeline
    ZED_custom_frame_protobuf_lib
    ${OpenCV_LIBS}
)


# ZEDRecPlay executable
set(VISUALIZER_LIBS
    Qt6::Core
    Qt6::Gui
    Qt6::OpenGL
    OpenGL::GL
    glm::glm
    Qt6::Widgets
    Qt6::OpenGLWidgets
)

add_executable(ZEDRecPlay
    ZEDRecPlay.cpp
)
target_include_directories(ZEDRecPlay PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_BINARY_DIR}/proto  # For generated protobuf headers
)
# Require C99 and C++17
target_compile_features(ZEDRecPlay PUBLIC
    c_std_99
    cxx_std_17
)
target_link_libraries(ZEDRecPlay
    Orbis_SLAM_pipeline
    ${VISUALIZER_LIBS}
    ZED_custom_frame_protobuf_lib
)




install(TARGETS 
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_playback_node
  ZEDRecPlay
  DESTINATION lib/${PROJECT_NAME}
)