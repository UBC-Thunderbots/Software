load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cost_functions",
    srcs = ["cost_function.cpp"],
    hdrs = ["cost_function.h"],
    deps = [
        ":pass",
        "//proto/message_translation:tbots_protobuf",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:time_to_travel",
        "//software/ai/passing:eighteen_zone_pitch_division",
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
        ":cost_functions",
        "//shared/test_util:tbots_gtest_main",
        "//software/math:math_functions",
        "//software/test_util",
    ],
)

cc_library(
    name = "pass",
    srcs = ["pass.cpp"],
    hdrs = ["pass.h"],
    deps = [
        "//shared:constants",
        "//software/time:timestamp",
        "//software/world:field",
    ],
)

cc_test(
    name = "pass_test",
    srcs = ["pass_test.cpp"],
    deps = [
        ":pass",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "pass_with_rating",
    srcs = ["pass_with_rating.cpp"],
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
        "//shared/test_util:tbots_gtest_main",
        "//software/geom/algorithms",
    ],
)

cc_library(
    name = "receiver_position_generator",
    hdrs = [
        "receiver_position_generator.hpp",
    ],
    deps = [
        ":cost_functions",
        ":field_pitch_division",
        ":pass",
        ":pass_with_rating",
        "//software/geom:point",
        "//software/geom:rectangle",
        "//software/util/make_enum",
        "//software/world",
    ],
)

cc_test(
    name = "receiver_position_generator_test",
    srcs = ["receiver_position_generator_test.cpp"],
    deps = [
        ":eighteen_zone_pitch_division",
        ":receiver_position_generator",
        "//shared/test_util:tbots_gtest_main",
        "//software/math:math_functions",
        "//software/test_util",
    ],
)

cc_library(
    name = "pass_generator",
    srcs = [
        "pass_generator.cpp",
    ],
    hdrs = ["pass_generator.h"],
    deps = [
        ":cost_functions",
        ":pass_feature_collector",
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
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)

cc_library(
    name = "pass_feature_collector",
    srcs = ["pass_feature_collector.cpp"],
    hdrs = ["pass_feature_collector.h"],
    deps = [
        ":cost_functions",
        ":pass",
        "//software/ai/evaluation:calc_best_shot",
        "//software/world",
        "//software/logger:generic_logger"
    ],
)
