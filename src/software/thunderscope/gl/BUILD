package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "gl_widget",
    srcs = ["gl_widget.py"],
    deps = [
        ":gl_layer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_layer",
    srcs = ["gl_layer.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_path_layer",
    srcs = ["gl_path_layer.py"],
    deps = [
        ":gl_layer",
        "//software/thunderscope/gl/graphics:gl_robot_outline",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_obstacle_layer",
    srcs = ["gl_obstacle_layer.py"],
    deps = [
        ":gl_layer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_passing_layer",
    srcs = ["gl_passing_layer.py"],
    deps = [
        ":gl_layer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_world_layer",
    srcs = ["gl_world_layer.py"],
    deps = [
        ":gl_layer",
        "//software/thunderscope/gl/graphics:gl_ball",
        "//software/thunderscope/gl/graphics:gl_circle",
        "//software/thunderscope/gl/graphics:gl_rect",
        "//software/thunderscope/gl/graphics:gl_robot",
        requirement("pyqtgraph"),
    ],
)
