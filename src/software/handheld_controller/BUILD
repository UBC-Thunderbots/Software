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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "controller",
    srcs = ["controller.cpp"],
    hdrs = ["controller.h"],
    deps = [
        ":controller_input",
        "//shared/parameter:cpp_configs",
        "//software/logger",
        "//software/math:math_functions",
        "//software/multithreading:subject",
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
        "//proto/message_translation:tbots_protobuf",
        "//proto/primitive:primitive_msg_factory",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
    ],
)

cc_test(
    name = "controller_primitive_generator_test",
    srcs = ["controller_primitive_generator_test.cpp"],
    deps = [
        ":controller_primitive_generator",
        "//shared/parameter:cpp_configs",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
