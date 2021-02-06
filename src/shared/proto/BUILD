load("@rules_proto//proto:defs.bzl", "proto_library")
load("//shared/proto:nanopb.bzl", "nanopb_proto_library")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "tbots_proto",
    srcs = [
        "geometry.proto",
        "primitive.proto",
        "robot_log_msg.proto",
        "robot_status_msg.proto",
        "tbots_software_msgs.proto",
        "tbots_timestamp_msg.proto",
        "vision.proto",
    ],
    deps = [
        "@nanopb//:nanopb_options_proto",
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

py_proto_library(
    name = "tbots_py_proto",
    srcs = [
        "geometry.proto",
        "primitive.proto",
        "robot_status_msg.proto",
        "tbots_software_msgs.proto",
        "tbots_timestamp_msg.proto",
        "vision.proto",
    ],
    deps = [
        "@nanopb//:nanopb_options_py_proto",
    ],
)
