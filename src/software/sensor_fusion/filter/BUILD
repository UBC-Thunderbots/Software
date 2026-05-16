load("@simulated_tests_deps//:requirements.bzl", "requirement")

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
    name = "kalman_filter",
    srcs = ["kalman_filter.cpp"],
    hdrs = ["kalman_filter.h"],
    deps = [
        "@eigen",
    ],
)
cc_library(
    name = "ball_tracker",
    srcs = ["ball_tracker.cpp"],
    hdrs = ["ball_tracker.h"],
    deps = [
        ":vision_detection",
		":kalman_filter",
        "//software/geom/algorithms",
        "//software/math:math_functions",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:robot",
        "//software:constants",
        "@boost//:circular_buffer",
        "@eigen",
    ],
)

cc_test(
    name = "ball_tracker_test",
    srcs = ["ball_tracker_test.cpp"],
    deps = [
        ":ball_tracker",
        "//shared/test_util:tbots_gtest_main",
        "//software/world:field",
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
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "sensor_fusion_filters",
    deps = [
        ":ball_tracker",
        ":robot_team_filter",
    ],
)
