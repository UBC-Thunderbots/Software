package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controller_input",
    srcs = ["controller_input.cpp"],
    hdrs = ["controller_input.h"],
    deps = [],
)

cc_test(
    name = "controller_input_test",
    srcs = ["controller_input_test.cpp"],
    deps = [
        ":controller_input",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "controller",
    srcs = ["controller.cpp"],
    hdrs = ["controller.h"],
    deps = [
        ":controller_input",
        "//software/logger",
        "//software/math:math_functions",
        "//software/multithreading:subject",
        "//software/parameter:dynamic_parameters",
    ],
)

cc_library(
    name = "xbox_360_controller",
    srcs = ["xbox_360_controller.cpp"],
    hdrs = ["xbox_360_controller.h"],
    deps = [":controller"],
)

cc_library(
    name = "controller_primitive_generator",
    srcs = ["controller_primitive_generator.cpp"],
    hdrs = ["controller_primitive_generator.h"],
    deps = [
        ":controller",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/primitive",
        "//software/primitive:chip_primitive",
        "//software/primitive:direct_velocity_primitive",
        "//software/primitive:kick_primitive",
    ],
)

cc_binary(
    name = "handheld_control",
    srcs = ["handheld_control_main.cpp"],
    deps = [
        ":controller_primitive_generator",
        ":xbox_360_controller",
        "//software/backend:radio_backend",
        "//software/parameter:dynamic_parameters",
    ],
)
