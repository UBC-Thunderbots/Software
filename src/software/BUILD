package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "full_system",
    srcs = ["full_system_main.cpp"],
    deps = [
        "//proto/logging:proto_logger",
        "//proto/message_translation:ssl_wrapper",
        "//shared/parameter:cpp_configs",
        "//software:constants",
        "//software/ai:threaded_ai",
        "//software/ai/hl/stp:play_info",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/estop:arduino_util",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//software/multithreading:observer_subject_adapter",
        "//software/sensor_fusion:threaded_sensor_fusion",
        "//software/util/design_patterns:generic_factory",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "standalone_simulator_main",
    srcs = ["standalone_simulator_main.cpp"],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/gui/standalone_simulator:threaded_standalone_simulator_gui",
        "//software/logger",
        "//software/simulation:standalone_simulator",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "handheld_control",
    srcs = ["handheld_control_main.cpp"],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/backend:radio_backend",
        "//software/handheld_controller:controller_primitive_generator",
        "//software/handheld_controller:xbox_360_controller",
    ],
)

cc_binary(
    name = "robot_diagnostics_main",
    srcs = ["robot_diagnostics_main.cpp"],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/gui/robot_diagnostics:threaded_robot_diagnostics_gui",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "network_log_listener_main",
    srcs = ["network_log_listener_main.cpp"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/logger:network_logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
)

sh_binary(
    name = "restart_on_crash",
    srcs = ["restart_on_crash.sh"],
    deps = [
        "@bazel_tools//tools/bash/runfiles",
    ],
)
