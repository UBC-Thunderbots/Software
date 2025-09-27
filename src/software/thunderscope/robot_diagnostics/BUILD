load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

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
    name = "handheld_controller",
    srcs = ["handheld_controller.py"],
)

py_library(
    name = "handheld_controller_widget",
    srcs = ["handheld_controller_widget.py"],
    deps = [
        ":handheld_controller",
        "//software/thunderscope:constants",
        requirement("pyqtgraph"),
        requirement("evdev"),
    ],
)

py_library(
    name = "diagnostics_widget",
    srcs = ["diagnostics_widget.py"],
    deps = [
        "//software/logger:py_logger",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        "//software/thunderscope:thunderscope_types",
        "//software/thunderscope/robot_diagnostics:chicker",
        "//software/thunderscope/robot_diagnostics:drive_and_dribbler_widget",
        "//software/thunderscope/robot_diagnostics:handheld_controller_widget",
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
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "estop_view",
    srcs = ["estop_view.py"],
    deps = [
        "//proto:estop_state_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "robot_status",
    srcs = ["robot_status.py"],
)

py_library(
    name = "robot_error_log",
    srcs = ["robot_error_log.py"],
    deps = [
        ":error_log_widgets",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "error_log_widgets",
    srcs = ["error_log_widgets.py"],
    deps = [
        ":robot_status",
        "//software/thunderscope/robot_diagnostics/robot_error_log_icons:error_log_constants",
        requirement("pyqtgraph"),
    ],
)
