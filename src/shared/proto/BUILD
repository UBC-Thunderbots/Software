load("@rules_proto//proto:defs.bzl", "proto_lang_toolchain", "proto_library")
load(":nanopb.bzl", "c_proto_library")

package(default_visibility = ["//visibility:public"])

proto_files = [
    "camera.proto",
    "drive.proto",
    "control.proto",
    "primitive.proto",
]

# proto files
proto_library(
    name = "fw_proto",
    srcs = proto_files,
    visibility = ["//visibility:private"],
)

# TODO add ctx to macro
# https://github.com/UBC-Thunderbots/Software/issues/1133
c_proto_library(
    name = "fw_c_proto",
    srcs = proto_files,
    restricted_to = [
        "//cc_toolchain:stm32f4",
        "//cc_toolchain:stm32h7",
    ],
)

cc_proto_library(
    name = "fw_cc_proto",
    deps = [":fw_proto"],
)
