package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "gl_circle",
    srcs = ["gl_circle.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_rect",
    srcs = ["gl_rect.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_robot_outline",
    srcs = ["gl_robot_outline.py"],
    deps = [
        requirement("pyqtgraph"),
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
    name = "gl_ball",
    srcs = ["gl_ball.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)