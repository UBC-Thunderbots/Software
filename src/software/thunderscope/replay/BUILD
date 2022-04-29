package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "replay_controls",
    srcs = ["replay_controls.py"],
    deps = [
        "//software/thunderscope:common_widgets",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "replay",
    srcs = ["replay.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)
