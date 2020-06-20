load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "standalone_simulator_gui_wrapper",
    srcs = ["standalone_simulator_gui_wrapper.cpp"],
    hdrs = ["standalone_simulator_gui_wrapper.h"],
    deps = [
        "//software/gui/standalone_simulator/widgets:simulator_gui",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/new_geom:rectangle",
        "//software/proto:ssl_cc_proto",
        "//software/proto/message_translation:ssl_geometry",
        "@qt//:qt_widgets",
    ],
)
