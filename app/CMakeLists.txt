add_executable(${PROJECT_NAME}_node
    orbis_slam_main.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Require C99 and C++17
target_compile_features(${PROJECT_NAME}_node  PUBLIC 
    c_std_99 
    cxx_std_17
)
target_link_libraries(${PROJECT_NAME}_node
    Orbis_SLAM_pipeline
)