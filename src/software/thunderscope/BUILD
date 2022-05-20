package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_binary(
    name = "thunderscope_main",
    srcs = ["thunderscope_main.py"],
    deps = [
        ":thunderscope",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope/replay:proto_logger",
    ],
)

py_library(
    name = "thunderscope",
    srcs = ["thunderscope.py"],
    data = [
        "//software:er_force_simulator_main",
        "//software:py_constants.so",
        "//software:python_bindings.so",
        "//software:unix_full_system",
    ],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:software_py_proto",
        "//proto/message_translation:py_tbots_protobuf",
        "//software/networking:threaded_unix_listener_py",
        "//software/networking:threaded_unix_sender_py",
        "//software/thunderscope:binary_context_managers",
        "//software/thunderscope:colors",
        "//software/thunderscope:common_widgets",
        "//software/thunderscope:constants",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope:robot_communication",
        "//software/thunderscope/arbitrary_plot:proto_plotter",
        "//software/thunderscope/arbitrary_plot:named_value_plotter",
        "//software/thunderscope/common:proto_configuration_widget",
        "//software/thunderscope/field",
        "//software/thunderscope/field:field_layer",
        "//software/thunderscope/field:obstacle_layer",
        "//software/thunderscope/field:passing_layer",
        "//software/thunderscope/field:path_layer",
        "//software/thunderscope/field:simulator_layer",
        "//software/thunderscope/field:validation_layer",
        "//software/thunderscope/field:world_layer",
        "//software/thunderscope/log:g3log_checkboxes",
        "//software/thunderscope/log:g3log_widget",
        "//software/thunderscope/play:playinfo_widget",
        "//software/thunderscope/replay:proto_logger",
        "//software/thunderscope/replay:proto_player",
        "//software/thunderscope/robot_diagnostics:chicker",
        "//software/thunderscope/robot_diagnostics:drive_and_dribbler_widget",
        requirement("pyqtgraph"),
        requirement("numpy"),
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
    name = "colors",
    srcs = ["colors.py"],
)

py_library(
    name = "common_widgets",
    srcs = ["common_widgets.py"],
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
