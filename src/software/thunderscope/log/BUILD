package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "g3log_widget",
    srcs = ["g3log_widget.py"],
    deps = [
        "//proto:software_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "g3log_checkboxes",
    srcs = ["g3log_checkboxes.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)
