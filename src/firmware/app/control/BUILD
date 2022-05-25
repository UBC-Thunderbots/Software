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
        "trajectory_planner_private.c",
        "trajectory_planner_private.h",
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
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:vector",
    ],
)

cc_library(
    name = "wheel_controller",
    srcs = ["wheel_controller.c"],
    hdrs = ["wheel_controller.h"],
    deps = [
        "//firmware/shared:circular_buffer",
    ],
)

cc_test(
    name = "wheel_controller_test",
    srcs = [
        "wheel_controller_test.cpp",
    ],
    deps = [
        ":wheel_controller",
        "//firmware/shared:circular_buffer",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "trajectory_planner_private_test",
    srcs = [
        "trajectory_planner_private.h",
        "trajectory_planner_private_test.cpp",
    ],
    deps = [
        ":trajectory_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:vector",
    ],
)

cc_library(
    name = "control",
    srcs = ["control.c"],
    hdrs = ["control.h"],
    deps = [
        "//firmware/app/world:force_wheel",
        "//firmware/shared:physics",
        "//firmware/shared:util",
        "//shared:robot_constants",
    ],
)

cc_library(
    name = "physbot",
    srcs = ["physbot.c"],
    hdrs = ["physbot.h"],
    deps = [
        ":bangbang",
        "//firmware/shared:physics",
    ],
)
