package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bangbang",
    srcs = ["bangbang.c"],
    hdrs = ["bangbang.h"],
    deps = [],
)

cc_library(
    name = "trajectory_planner",
    srcs = [
        "trajectory_planner.c",
        "trajectory_planner_impl.c",
        "trajectory_planner_impl.h",
    ],
    hdrs = ["trajectory_planner.h"],
    deps = [
        "//firmware/shared:physics",
        "//firmware/shared/math:polynomial_1d",
        "//firmware/shared/math:polynomial_2d",
        "//firmware/shared/math:tbots_math",
        "//firmware/shared/math:vector_2d",
    ],
)

cc_test(
    name = "trajectory_planner_test",
    srcs = [
        "trajectory_planner_test.cpp",
    ],
    deps = [
        ":trajectory_planner",
        "//software/new_geom:vector",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_planner_impl_test",
    srcs = [
        "trajectory_planner_impl.h",
        "trajectory_planner_impl_test.cpp",
    ],
    deps = [
        ":trajectory_planner",
        "//software/new_geom:vector",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "control",
    srcs = ["control.c"],
    hdrs = ["control.h"],
    deps = [
        "//firmware/app/world:firmware_robot",
        "//firmware/shared:physics",
        "//firmware/shared:util",
    ],
)

cc_library(
    name = "physbot",
    srcs = ["physbot.c"],
    hdrs = ["physbot.h"],
    deps = [
        ":bangbang",
        "//firmware/app/world:firmware_robot",
        "//firmware/shared:physics",
    ],
)
