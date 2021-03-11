package(default_visibility = ["//visibility:public"])

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")

cc_library(
    name = "evaluation",
    srcs = ["cost_function.cpp"],
    hdrs = ["cost_function.h"],
    deps = [
        ":pass",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:pass",
        "//software/logger",
        "//software/math:math_functions",
        "//software/util/make_enum",
        "//software/world",
    ],
)

cc_test(
    name = "evaluation_test",
    srcs = ["cost_function_test.cpp"],
    deps = [
        ":evaluation",
        "//software/math:math_functions",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "pass",
    srcs = ["pass.cpp"],
    hdrs = ["pass.h"],
    deps = [
        "//software/time:timestamp",
        "//software/world:field",
    ],
)

cc_test(
    name = "pass_test",
    srcs = ["pass_test.cpp"],
    deps = [
        ":pass",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "pass_with_rating",
    hdrs = ["pass_with_rating.h"],
    deps = [
        ":pass",
    ],
)

cc_library(
    name = "pass_generator",
    srcs = ["pass_generator.cpp"],
    hdrs = ["pass_generator.h"],
    deps = [
        ":evaluation",
        ":pass",
        ":pass_with_rating",
        "//software/optimization:gradient_descent",
        "//software/world",
    ],
)

cc_test(
    name = "pass_generator_test",
    srcs = ["pass_generator_test.cpp"],
    deps = [
        ":pass_generator",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
