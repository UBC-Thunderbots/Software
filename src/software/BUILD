package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "full_system",
    srcs = ["full_system_main.cpp"],
    deps = [
        "//software:constants",
        "//software/ai:threaded_ai",
        "//software/ai/hl/stp:play_info",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//software/multithreading:observer_subject_adapter",
        "//software/parameter:dynamic_parameters",
        "//software/proto/logging:proto_logger",
        "//software/proto/message_translation:ssl_wrapper",
        "//software/sensor_fusion:threaded_sensor_fusion",
        "//software/util/design_patterns:generic_factory",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "standalone_simulator_main",
    srcs = ["standalone_simulator_main.cpp"],
    deps = [
        "//simulation:standalone_simulator",
        "//software/gui/standalone_simulator:threaded_standalone_simulator_gui",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "handheld_control",
    srcs = ["handheld_control_main.cpp"],
    deps = [
        "//handheld_controller:controller_primitive_generator",
        ":xbox_360_controller",
        "//software/backend:radio_backend",
        "//software/parameter:dynamic_parameters",
    ],
)

cc_binary(
    name = "robot_diagnostics_main",
    srcs = ["robot_diagnostics_main.cpp"],
    deps = [
        "//robot_diagnostics:threaded_robot_diagnostics_gui",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
)
