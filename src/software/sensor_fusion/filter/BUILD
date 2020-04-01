package(default_visibility = ["//software/sensor_fusion:__subpackages__"])

cc_library(
    name = "ball_filter",
    srcs = ["ball_filter.cpp"],
    hdrs = ["ball_filter.h"],
    deps = [
        "//software/geom",
        "//software/math:math_functions",
        "//software/new_geom/util",
        "//software/sensor_fusion:vision_detection",
        "//software/world:ball",
        "//software/world:field",
        "@eigen",
    ],
)

cc_test(
    name = "ball_filter_test",
    srcs = ["ball_filter_test.cpp"],
    deps = [
        ":ball_filter",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot_filter",
    srcs = ["robot_filter.cpp"],
    hdrs = ["robot_filter.h"],
    deps = [
        "//software/geom",
        "//software/sensor_fusion:vision_detection",
        "//software/world:robot",
    ],
)

cc_test(
    name = "robot_filter_test",
    srcs = ["robot_filter_test.cpp"],
    deps = [
        ":robot_filter",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot_team_filter",
    srcs = ["robot_team_filter.cpp"],
    hdrs = ["robot_team_filter.h"],
    deps = [
        ":robot_filter",
        "//software:constants",
        "//software/world:team",
    ],
)

cc_test(
    name = "robot_team_filter_test",
    srcs = ["robot_team_filter_test.cpp"],
    deps = [
        ":robot_team_filter",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "sensor_fusion_filters",
    deps = [
        ":ball_filter",
        ":robot_team_filter",
    ],
)
