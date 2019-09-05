# Find Protobuf
find_package(Protobuf REQUIRED)

## Call our CMake file to include and build protobuf
include("${CMAKE_CURRENT_SOURCE_DIR}/build_proto.cmake")

## Find the generated files
file(GLOB TBOTS_PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.pb.cc")

add_library(tbots_proto STATIC
    ${TBOTS_PROTO_SRCS}
    )
target_link_libraries(tbots_proto 
    ${PROTOBUF_LIBRARIES}
    )
