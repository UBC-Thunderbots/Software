package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velocity_obstacle",
    srcs = ["velocity_obstacle.cpp"],
    hdrs = [
        "velocity_obstacle.h",
    ],
    deps = [
        "//software/geom:vector",
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
