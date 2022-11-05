package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velocity_obstacle",
    srcs = ["velocity_obstacle.cpp"],
    hdrs = [
        "velocity_obstacle.h",
    ],
    deps = [
        "//software/geom:circle",
        "//software/geom:vector",
        "//software/logger",
    ],
)

cc_test(
    name = "velocity_obstacle_test",
    srcs = ["velocity_obstacle_test.cpp"],
    deps = [
        "velocity_obstacle",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "fixed_radius_nearest_neighbor_search",
    srcs = [
        "brute_force_nearest_neighbor_search.cpp",
    ],
    hdrs = [
        "brute_force_nearest_neighbor_search.h",
    ],
)

cc_test(
    name = "nearest_neighbor_test",
    srcs = ["nearest_neighbor_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/path_planner/hrvo:brute_force_nearest_neighbor_search",
        "//software/test_util",
        "//software/time:duration",
    ],
)
