package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "drive_and_dribbler_widget",
    srcs = ["drive_and_dribbler_widget.py"],
    deps = [
        "//proto:software_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope/common:common_widgets",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "chicker",
    srcs = ["chicker_widget.py"],
    deps = [
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "fullsystem_connect_widget",
    srcs = ["fullsystem_connect_widget.py"],
    deps = [
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "robot_view",
    srcs = ["robot_view.py"],
    deps = [
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "estop_view",
    srcs = ["estop_view.py"],
    deps = [
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)
