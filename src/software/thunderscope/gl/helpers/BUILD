package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "graphics_view",
    srcs = ["graphics_view.py"],
    deps = [
        ":extended_gl_view_widget",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "extended_gl_view_widget",
    srcs = ["extended_gl_view_widget.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)
