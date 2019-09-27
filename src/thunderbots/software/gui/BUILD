load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visualizer_wrapper",
    srcs = ["visualizer_wrapper.cpp"],
    hdrs = ["visualizer_wrapper.h"],
    deps = [
        "//software/ai/hl/stp:play_info",
        "//software/ai/world",
        "//software/backend:robot_status",
        "//software/gui/drawing:draw_functions",
        "//software/gui/drawing:world",
        "//software/gui/widgets:visualizer",
        "//software/multithreading:threaded_observer",
        "@qt//:qt_widgets",
    ],
)
