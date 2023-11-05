package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "gl_circle",
    srcs = ["gl_circle.py"],
    deps = [
        requirement("pyqtgraph"),
        ":gl_shape",
    ],
)

py_library(
    name = "gl_rect",
    srcs = ["gl_rect.py"],
    deps = [
        requirement("pyqtgraph"),
        ":gl_shape",
    ],
)

py_library(
    name = "gl_robot_outline",
    srcs = ["gl_robot_outline.py"],
    deps = [
        requirement("pyqtgraph"),
        ":gl_shape",
        "//software/thunderscope/gl/helpers:triangulate",
    ],
)

py_library(
    name = "gl_robot",
    srcs = ["gl_robot.py"],
    deps = [
        ":gl_robot_outline",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_sphere",
    srcs = ["gl_sphere.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_goal",
    srcs = ["gl_goal.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_polygon",
    srcs = ["gl_polygon.py"],
    deps = [
        requirement("pyqtgraph"),
        ":gl_shape",
        "//software/thunderscope/gl/helpers:triangulate",
    ],
)

py_library(
    name = "gl_shape",
    srcs = ["gl_shape.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_heatmap",
    srcs = ["gl_heatmap.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)
