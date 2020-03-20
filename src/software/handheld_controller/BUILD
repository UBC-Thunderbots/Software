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
    name = "primitive_generator",
    srcs = ["primitive_generator.cpp"],
    hdrs = ["primitive_generator.h"],
    deps = [
        ":controller",
        "//software/ai/primitive",
        "//software/ai/primitive:chip_primitive",
        "//software/ai/primitive:direct_velocity_primitive",
        "//software/ai/primitive:kick_primitive",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
    ],
)

cc_binary(
    name = "handheld_control",
    srcs = ["handheld_control_main.cpp"],
    deps = [
        ":primitive_generator",
        ":xbox_360_controller",
        "//software/backend:handheld_controller_backend",
        "//software/parameter:dynamic_parameters",
    ],
)
