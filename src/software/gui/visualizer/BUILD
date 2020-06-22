load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visualizer_wrapper",
    srcs = ["visualizer_wrapper.cpp"],
    hdrs = ["visualizer_wrapper.h"],
    deps = [
        "//software/ai/hl/stp:play_info",
        "//software/backend:robot_status",
        "//software/geom",
        "//software/gui/drawing:draw_functions",
        "//software/gui/drawing:world",
        "//software/gui/visualizer/widgets:visualizer",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/proto:sensor_msg_cc_proto",
        "//software/world",
        "@qt//:qt_widgets",
    ],
)
