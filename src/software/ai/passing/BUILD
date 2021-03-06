package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cost_functions",
    srcs = ["cost_function.cpp"],
    hdrs = ["cost_function.h"],
    deps = [
        ":pass",
        "//software/ai/evaluation:pass",
        "//software/logger",
        "//software/math:math_functions",
        "//software/parameter:dynamic_parameters",
        "//software/util/make_enum",
        "//software/world",
    ],
)

cc_test(
    name = "evaluation_test",
    srcs = ["cost_function_test.cpp"],
    deps = [
        ":cost_functions",
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
    name = "field_pitch_division",
    hdrs = ["field_pitch_division.h"],
    deps = [
        "//software/geom:point",
        "//software/geom:rectangle",
        "//software/world:field",
    ],
)

cc_library(
    name = "eighteen_zone_pitch_division",
    srcs = [
        "eighteen_zone_pitch_division.cpp",
    ],
    hdrs = ["eighteen_zone_pitch_division.h"],
    deps = [
        ":field_pitch_division",
        "//software/geom:point",
        "//software/geom:rectangle",
        "//software/util/make_enum",
    ],
)

cc_test(
    name = "eighteen_zone_pitch_division_test",
    srcs = ["eighteen_zone_pitch_division_test.cpp"],
    deps = [
        ":eighteen_zone_pitch_division",
        "//software/geom/algorithms",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "pass_evaluation",
    srcs = [
        "pass_evaluation.cpp",
    ],
    hdrs = ["pass_evaluation.h"],
    deps = [
        ":field_pitch_division",
        ":pass",
        ":pass_with_rating",
        "//software/geom:point",
        "//software/geom:rectangle",
        "//software/util/make_enum",
    ],
)

cc_test(
    name = "pass_evaluation_test",
    srcs = ["pass_evaluation_test.cpp"],
    deps = [
        ":eighteen_zone_pitch_division",
        ":pass",
        ":pass_evaluation",
        "//software/geom/algorithms",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "pass_generator",
    srcs = ["pass_generator.cpp"],
    hdrs = ["pass_generator.h"],
    deps = [
        ":cost_functions",
        ":pass",
        ":pass_evaluation",
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
