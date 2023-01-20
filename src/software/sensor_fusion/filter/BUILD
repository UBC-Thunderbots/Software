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
        "@boost//:circular_buffer",
        "@eigen",
    ],
)

cc_test(
    name = "ball_filter_test",
    srcs = ["ball_filter_test.cpp"],
    deps = [
        ":ball_filter",
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
        ":ball_filter",
        ":robot_team_filter",
    ],
)

cc_test(
    name = "ball_occlusion_test",
    srcs = ["ball_occlusion_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:halt_play",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
