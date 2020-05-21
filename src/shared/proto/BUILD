load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "tbots_proto",
    srcs = [
        "robot_component_status.proto",
        "tbots_robot_msg.proto",
    ],
    # Note: this library is public so that proto libraries in firmware or software can build from it
)

cc_proto_library(
    name = "tbots_cc_proto",
    deps = [":tbots_proto"],
)
