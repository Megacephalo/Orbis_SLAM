# Generate protobuf C++ sources
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS zed_recording.proto)

# Create SHARED library from generated sources (needed for -fPIC)
add_library(ZED_custom_frame_protobuf_lib SHARED
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

# Enable position-independent code
set_target_properties(ZED_custom_frame_protobuf_lib PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(ZED_custom_frame_protobuf_lib PUBLIC
    protobuf::libprotobuf
)

# Include both source and binary directories for generated headers
target_include_directories(ZED_custom_frame_protobuf_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Install the protobuf library
install(TARGETS ZED_custom_frame_protobuf_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)