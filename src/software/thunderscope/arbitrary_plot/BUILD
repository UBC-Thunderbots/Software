package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "named_value_plotter",
    srcs = ["named_value_plotter.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)
