package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_binary(
    name = "thunderscope_main",
    srcs = ["thunderscope_main.py"],
    deps = [
        ":config",
        ":constants",
        ":thunderscope",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope/replay:proto_logger",
    ],
)

py_library(
    name = "thunderscope",
    srcs = ["thunderscope.py"],
    data = [
        "thunderscope-logo.png",
        "//software:er_force_simulator_main",
        "//software:py_constants.so",
        "//software:python_bindings.so",
        "//software:unix_full_system",
    ],
    deps = [
        ":config",
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:software_py_proto",
        "//proto/message_translation:py_tbots_protobuf",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope:constants",
        "//software/thunderscope:dock_label_style",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope:robot_communication",
        requirement("pyqtgraph"),
        requirement("numpy"),
    ],
)

py_library(
    name = "widget_names_to_setup",
    srcs = ["widget_setup_functions.py"],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    deps = [
        "//software/thunderscope/common:common_widgets",
        "//software/thunderscope/common:proto_configuration_widget",
        "//software/thunderscope/common:proto_plotter",
        "//software/thunderscope/cost_vis",
        "//software/thunderscope/field",
        "//software/thunderscope/field:field_layer",
        "//software/thunderscope/field:hrvo_layer",
        "//software/thunderscope/field:obstacle_layer",
        "//software/thunderscope/field:passing_layer",
        "//software/thunderscope/field:path_layer",
        "//software/thunderscope/field:simulator_layer",
        "//software/thunderscope/field:validation_layer",
        "//software/thunderscope/field:world_layer",
        "//software/thunderscope/gl:gl_obstacle_layer",
        "//software/thunderscope/gl:gl_passing_layer",
        "//software/thunderscope/gl:gl_path_layer",
        "//software/thunderscope/gl:gl_widget",
        "//software/thunderscope/gl:gl_world_layer",
        "//software/thunderscope/gl:gl_hrvo_layer",
        "//software/thunderscope/log:g3log_checkboxes",
        "//software/thunderscope/log:g3log_widget",
        "//software/thunderscope/play:playinfo_widget",
        "//software/thunderscope/play:refereeinfo_widget",
        "//software/thunderscope/replay:proto_logger",
        "//software/thunderscope/replay:proto_player",
        "//software/thunderscope/robot_diagnostics:chicker",
        "//software/thunderscope/robot_diagnostics:diagnostics_input_widget",
        "//software/thunderscope/robot_diagnostics:drive_and_dribbler_widget",
        "//software/thunderscope/robot_diagnostics:estop_view",
        "//software/thunderscope/robot_diagnostics:robot_info",
        "//software/thunderscope/robot_diagnostics:robot_view",
    ],
)

py_library(
    name = "thunderscope_types",
    srcs = ["thunderscope_types.py"],
    deps = [
        ":constants",
    ],
)

py_library(
    name = "config",
    srcs = ["thunderscope_config.py"],
    deps = [
        ":constants",
        ":proto_unix_io",
        ":thunderscope_types",
        ":widget_names_to_setup",
    ],
)

py_library(
    name = "proto_unix_io",
    srcs = ["proto_unix_io.py"],
)

py_library(
    name = "constants",
    srcs = ["constants.py"],
)

py_library(
    name = "dock_label_style",
    srcs = ["dock_label_style.py"],
)

py_library(
    name = "thread_safe_buffer",
    srcs = ["thread_safe_buffer.py"],
)

py_library(
    name = "binary_context_managers",
    srcs = ["binary_context_managers.py"],
)

py_library(
    name = "robot_communication",
    srcs = ["robot_communication.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//software/thunderscope:constants",
    ],
)
