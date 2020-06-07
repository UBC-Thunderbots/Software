load("@rules_proto//proto:defs.bzl", "proto_library")
load("//firmware_new:nanopb.bzl", "nanopb_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "fw_proto",
    srcs = ["control.proto"],
    visibility = ["//visibility:private"],
)

nanopb_proto_library(
    name = "fw_c_proto",
    deps = [
        ":fw_proto",
    ],
)

cc_proto_library(
    name = "fw_cc_proto",
    deps = [":fw_proto"],
)
