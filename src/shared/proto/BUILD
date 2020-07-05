load("@rules_proto//proto:defs.bzl", "proto_library")
load("//shared/proto:nanopb.bzl", "nanopb_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "tbots_proto",
    srcs = [
        "geometry.proto",
        "primitive.proto",
        "robot_component_status.proto",
        "tbots_robot_msg.proto",
        "tbots_software_msgs.proto",
        "tbots_timestamp_msg.proto",
        "vision.proto",
    ],
)

cc_proto_library(
    name = "tbots_cc_proto",
    deps = [":tbots_proto"],
)

nanopb_proto_library(
    name = "tbots_nanopb_proto",
    deps = [":tbots_proto"],
)
