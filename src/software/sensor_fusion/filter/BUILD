package(default_visibility = ["//software/sensor_fusion:__subpackages__"])

cc_library(
    name = "vision_detection",
    hdrs = [
        "vision_detection.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//software/geom:angle",
        "//software/geom:point",
        "//software/time:timestamp",
    ],
)

cc_library(
    name = "ball_filter",
    srcs = ["ball_filter.cpp"],
    hdrs = ["ball_filter.h"],
    deps = [
        ":vision_detection",
        "//software/geom/algorithms",
        "//software/math:math_functions",
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
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot_filter",
    srcs = ["robot_filter.cpp"],
    hdrs = ["robot_filter.h"],
    deps = [
        ":vision_detection",
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
    name = "possession_filter",
    srcs = ["possession_filter.cpp"],
    hdrs = ["possession_filter.h"],
    deps = [
        ":vision_detection",
        "//software:constants",
        "//software/world:ball",
        "//software/world:robot_state",
        "//software/world:team",
        "//software/world:timestamped_possession_state",
    ],
)

cc_test(
    name = "possession_filter_test",
    srcs = ["possession_filter_test.cpp"],
    deps = [
        ":possession_filter",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "sensor_fusion_filters",
    deps = [
        ":ball_filter",
        ":possession_filter",
        ":robot_team_filter",
    ],
)
