load("@rules_proto//proto:defs.bzl", "proto_library")
load(":nanopb.bzl", "c_proto_library")

package(default_visibility = ["//visibility:public"])

# TODO https://github.com/UBC-Thunderbots/Software/issues/1133
proto_files = [
    # "robot_component_status.proto",
    "tbots_robot_msg.proto",
    "tbots_msg.proto",
]

proto_library(
    name = "tbots_proto",
    srcs = proto_files,
    # Note: this library is public so that proto libraries in firmware or software can build from it
)

cc_proto_library(
    name = "tbots_cc_proto",
    deps = [":tbots_proto"],
)

c_proto_library(
    name = "tbots_c_proto",
    srcs = proto_files,
    restricted_to = ["//cc_toolchain:stm32h7"],
)
