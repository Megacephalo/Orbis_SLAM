add_executable(${PROJECT_NAME} 
    orbis_slam_main.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Require C99 and C++17
target_compile_features(${PROJECT_NAME}  PUBLIC 
    c_std_99 
    cxx_std_17
)