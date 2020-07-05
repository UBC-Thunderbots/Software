package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball_filter",
    srcs = ["ball_filter.cpp"],
    hdrs = ["ball_filter.h"],
    deps = [
        "//software/geom",
        "//software/math:math_functions",
        "//software/new_geom/util:algorithms",
        "//software/sensor_fusion/filter:vision_detection",
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
        "//software/sensor_fusion/filter:vision_detection",
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
