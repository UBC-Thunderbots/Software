package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ball_filter",
    srcs = ["ball_filter.cpp"],
    hdrs = ["ball_filter.h"],
    deps = [
        "//software/geom",
        "//software/util/math:math_functions",
        "//software/world:ball",
        "//software/world:field",
        "//software/sensor_fusion:detection",
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
        "//software/world:robot",
        "//software/sensor_fusion:detection",
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
        "//software/util:constants",
        "//software/world:team",
    ],
)
