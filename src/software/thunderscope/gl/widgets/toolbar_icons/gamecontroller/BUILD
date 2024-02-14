package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "icon_loader",
    srcs = ["icon_loader.py"],
    data = [
        ":blue.svg",
        ":force_start.svg",
        ":halt.svg",
        ":normal_start.svg",
        ":stop.svg",
        ":yellow.svg",
        "//software/thunderscope/gl/widgets:icon_loader",
    ],
    deps = [
        requirement("pyqtgraph"),
    ],
)
