load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

load("@rules_proto//proto:defs.bzl", "proto_library")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")

proto_library(
    name = "protobuf_proto_lib",
    srcs = glob([
        "*.proto",
    ]),
    visibility = ["//visibility:private"],
    deps = [
        "//proto:ssl_proto",
        "@com_google_protobuf//:any_proto",
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "protobuf_cc_lib",
    deps = [":protobuf_proto_lib"],
)

py_proto_library(
    name = "erforce_py_proto",
    srcs = glob([
        "*.proto",
    ]),
    deps = [
        "//proto:software_py_proto",
        "@com_google_protobuf//:protobuf_python",
    ],
)

qt_cc_library(
    name = "protobuf_qt",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        ":protobuf_cc_lib",
        "//proto:ssl_simulation_cc_proto",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)
