load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_python//python:proto.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "protobuf_proto_lib",
    srcs = glob([
        "*.proto",
    ]),
    visibility = ["//visibility:private"],
    deps = [
        "//proto:ssl_proto",
        "@protobuf//:any_proto",
        "@protobuf//:duration_proto",
        "@protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "protobuf_cc_lib",
    deps = [":protobuf_proto_lib"],
)

proto_library(
    name = "erforce_proto",
    srcs = glob([
        "*.proto",
    ]),
    deps = [
        "//proto:software_proto",
        "@protobuf//:any_proto",
    ],
)

py_proto_library(
    name = "erforce_py_proto",
    deps = [
        ":erforce_proto",
    ],
)

cc_library(
    name = "protobuf",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        ":protobuf_cc_lib",
        "//proto:ssl_simulation_cc_proto",
    ],
)
