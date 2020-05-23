load("@rules_proto//proto:defs.bzl", "proto_lang_toolchain", "proto_library")
load("@com_google_protobuf//:protobuf.bzl", "proto_gen")
load(":nanopb.bzl", "nanopb_proto_library")

package(default_visibility = ["//visibility:public"])

# add more proto here
proto_files = [
    "control.proto",
]

# proto files
proto_library(
    name = "fw_proto",
    srcs = proto_files,
    visibility = ["//visibility:private"],
)

# TODO add ctx to macro https://github.com/UBC-Thunderbots/Software/issues/1133
nanopb_proto_library(
    name = "fw_c_proto",
    nanopb_generator = "@nanopb//:nanopb_generator",
    nanopb_srcs = [
        "@nanopb//:nanopb_header",
    ],
    deps = [
        ":fw_proto",
    ],
)

# TODO: delete
#c_proto_library(
#    name = "fw_c_proto",
#    srcs = proto_files,
#    restricted_to = ["//cc_toolchain:stm32h7"],
#)

cc_proto_library(
    name = "fw_cc_proto",
    restricted_to = ["//cc_toolchain:k8"],
    deps = [":fw_proto"],
)
