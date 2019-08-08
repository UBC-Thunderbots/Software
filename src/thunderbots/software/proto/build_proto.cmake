find_package(Protobuf REQUIRED)

if(EXISTS ${PROTOBUF_PROTOC_EXECUTABLE})
    message(STATUS "Found PROTOBUF Compiler: ${PROTOBUF_PROTOC_EXECUTABLE}")
else()
    message(FATAL_ERROR "Could not find PROTOBUF Compiler")
endif()

# The folder where we put our compiled proto files
set("PROTO_OUTPUT_DIR" "${CMAKE_CURRENT_SOURCE_DIR}/proto")
# The source directory containing the uncompiled .proto files
set("PROTO_SOURCE_DIR" "${PROTO_OUTPUT_DIR}/src")

# Any existing compiled proto files
file(GLOB "EXISTING_COMPILED_PROTO" "${PROTO_OUTPUT_DIR}/*.pb.cc" "${PROTO_OUTPUT_DIR}/*.pb.h")

# Count the existing number of compiled proto files
set(EXISTING_COMPILED_PROTO_LENGTH)
list(LENGTH "EXISTING_COMPILED_PROTO" "EXISTING_COMPILED_PROTO_LENGTH")

## Remove any already-compiled files so we regerenerate them completely
#if ("${EXISTING_COMPILED_PROTO_LENGTH}" GREATER 0)
#    execute_process(COMMAND rm ${EXISTING_COMPILED_PROTO})
#endif()
#
## The proto source files that we need to compile
#file(GLOB "ProtoFiles" "${PROTO_SOURCE_DIR}/*.proto")
#
## Compile the proto files
#execute_process(
#        COMMAND protoc ${ProtoFiles}
#        --proto_path=${PROTO_SOURCE_DIR}
#        --cpp_out=${PROTO_OUTPUT_DIR})
