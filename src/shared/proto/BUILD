load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "tbots_proto",
    srcs = [
        "robot_component_status.proto",
        "tbots_robot_msg.proto",
    ],
    visibility = ["//visibility:private"],
)

cc_proto_library(
    name = "tbots_cc_proto",
    deps = [":tbots_proto"],
)
