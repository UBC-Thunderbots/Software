load("@thunderscope_deps//:requirements.bzl", "requirement")
load("@aspect_rules_py//py:defs.bzl", "py_library", "py_binary")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "playinfo_widget",
    srcs = ["playinfo_widget.py"],
    deps = [
        "//proto:software_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "refereeinfo_widget",
    srcs = ["refereeinfo_widget.py"],
    deps = [
        "//proto:software_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)
