load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "protobuf_proto_lib",
    srcs = glob([
        "*.proto",
    ]),
    visibility = ["//visibility:private"],
    deps = [
        "@com_google_protobuf//:any_proto",
        "@com_google_protobuf//:duration_proto",
        "@com_google_protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "protobuf_cc_lib",
    deps = [":protobuf_proto_lib"],
)

cc_library(
    name = "protobuf",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        ":protobuf_cc_lib",
        ":protobuf_qt",
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
