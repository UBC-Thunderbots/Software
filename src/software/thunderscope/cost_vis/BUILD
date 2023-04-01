package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "cost_vis",
    srcs = ["cost_vis.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)
