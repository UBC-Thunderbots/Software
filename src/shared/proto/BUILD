load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "tbots_proto",
    srcs = [
        "tbots_frankie_status.proto",
        "tbots_robot_msg.proto",
        "tbots_robot_status_2014.proto",
    ],
    visibility = ["//visibility:private"],
)

cc_proto_library(
    name = "tbots_cc_proto",
    deps = [":tbots_proto"],
)
