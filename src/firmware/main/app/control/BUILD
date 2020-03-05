package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bangbang",
    srcs = ["bangbang.c"],
    hdrs = ["bangbang.h"],
    deps = [],
)

cc_library(
    name = "trajectory_planner",
    srcs = ["trajectory_planner.c"],
    hdrs = ["trajectory_planner.h"],
    deps = [
        "//firmware/main/math:polynomial_1d",
        "//firmware/main/math:polynomial_2d",
        "//firmware/main/math:vector_2d",
    ],
)

cc_test(
    name = "trajectory_planner_test",
    srcs = ["trajectory_planner_test.cpp"],
    deps = [
        ":trajectory_planner",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "control",
    srcs = ["control.c"],
    hdrs = ["control.h"],
    deps = [
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/shared:physics",
        "//firmware/main/shared:util",
    ],
)

cc_library(
    name = "physbot",
    srcs = ["physbot.c"],
    hdrs = ["physbot.h"],
    deps = [
        ":bangbang",
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/shared:physics",
    ],
)
