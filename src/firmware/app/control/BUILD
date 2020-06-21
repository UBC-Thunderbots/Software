package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@python_control_deps//:requirements.bzl", "requirement")

py_binary(
    name = "generate_difference_equation",
    srcs = [":generate_difference_equation.py"],
    main = ":generate_difference_equation.py",
    deps = [
        requirement("control"),
    ],
)

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
        "//firmware/shared/math:polynomial_1d",
        "//firmware/shared/math:polynomial_2d",
        "//firmware/shared/math:vector_2d",
    ],
)

cc_test(
    name = "trajectory_planner_test",
    srcs = ["trajectory_planner_test.cpp"],
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
    name = "controller",
    srcs = ["controller.c"],
    hdrs = ["controller.h"],
    deps = [
        "//firmware/shared:circular_buffer",
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
