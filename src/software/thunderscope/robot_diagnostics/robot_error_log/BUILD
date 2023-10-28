package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "robot_error_log",
    srcs = ["robot_error_log.py"],
    data = [
        "crash.png",
        "error_code.png",
        "low_battery.png",
    ],
    deps = [
        ":error_log_constants",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "error_log_constants",
    srcs = ["error_log_constants.py"],
)
