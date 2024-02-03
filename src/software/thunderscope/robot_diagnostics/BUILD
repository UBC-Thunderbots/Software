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
    name = "controller",
    srcs = ["controller_diagnostics.py"],
    deps = [
        "//proto:software_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "diagnostics_widget",
    srcs = ["diagnostics_widget.py"],
    deps = [
        "//software/thunderscope/robot_diagnostics:controller",
        "//software/thunderscope/robot_diagnostics:chicker",
        "//software/thunderscope/robot_diagnostics:drive_and_dribbler_widget",
        "//software/thunderscope/robot_diagnostics:diagnostics_input_widget",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "diagnostics_input_widget",
    srcs = ["diagnostics_input_widget.py"],
    deps = [
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "robot_info",
    srcs = ["robot_info.py"],
    deps = [
        ":motor_fault_view",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "motor_fault_view",
    srcs = ["motor_fault_view.py"],
    deps = [
        "//software/thunderscope/common:common_widgets",
        "//software/thunderscope/common:proto_parameter_tree_util",
        "//software/thunderscope/robot_diagnostics/motor_fault_icons:motor_fault_icon_loader",
    ],
)

py_library(
    name = "robot_view",
    srcs = ["robot_view.py"],
    deps = [
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        "//software/thunderscope/robot_diagnostics:robot_info",
        "//software/thunderscope/robot_diagnostics:robot_status",
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

py_library(
    name = "robot_status",
    srcs = ["robot_status.py"],
)
