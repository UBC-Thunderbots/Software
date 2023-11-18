package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "gl_widget",
    srcs = ["gl_widget.py"],
    deps = [
        "//software/thunderscope/gl/helpers:extended_gl_view_widget",
        "//software/thunderscope/gl/layers:gl_layer",
        "//software/thunderscope/gl/layers:gl_measure_layer",
        "//software/thunderscope/replay:replay_controls",
        requirement("pyqtgraph"),
    ],
)
