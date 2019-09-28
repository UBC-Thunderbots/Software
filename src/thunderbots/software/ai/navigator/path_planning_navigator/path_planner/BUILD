package(default_visibility = ["//software/ai/navigator/path_planning_navigator:__subpackages__"])

cc_library(
    name = "path_planner",
    hdrs = ["path_planner.h"],
    deps = [
        "//software/ai/navigator/path_planning_navigator/obstacle",
        "//software/geom",
    ],
)

cc_library(
    name = "straight_line_path_planner",
    srcs = ["straight_line_path_planner.cpp"],
    hdrs = ["straight_line_path_planner.h"],
    deps = [":path_planner"],
)

cc_test(
    name = "straight_line_path_planner_test",
    srcs = ["straight_line_path_planner_test.cpp"],
    deps = [
        ":straight_line_path_planner",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "theta_star_path_planner",
    srcs = ["theta_star_path_planner.cpp"],
    hdrs = ["theta_star_path_planner.h"],
    deps = [
        ":path_planner",
        "//software/ai/world:ball",
        "//software/ai/world:field",
    ],
)

cc_test(
    name = "theta_star_path_planner_test",
    srcs = ["theta_star_path_planner_test.cpp"],
    deps = [
        ":theta_star_path_planner",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
