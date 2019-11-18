load("@rules_proto//proto:defs.bzl", "proto_library", "proto_lang_toolchain")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")
load(":nanopb.bzl", "c_proto_library")

package(default_visibility = ["//visibility:public"])

# proto files
proto_library(
    name = "fw_proto",
    srcs = [
        "robot.proto",
    ],
    visibility = ["//visibility:private"]
)

c_proto_library(
    name = "fw_c_proto",
    srcs = [
        "robot.proto",
    ],
)

