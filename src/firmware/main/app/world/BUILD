package(default_visibility = ["//visibility:public"])

# TODO: remove all `strip_include_prefix` once firmware has been setup to have includes
#       from top-level

cc_library(
    name = "chicker",
    srcs = ["chicker.c"],
    hdrs = ["chicker.h"],
    strip_include_prefix = "/firmware/main",
)

cc_test(
    name = "chicker_test",
    srcs = ["chicker_test.cpp"],
    deps = [
        ":chicker",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "dribbler",
    srcs = ["dribbler.c"],
    hdrs = ["dribbler.h"],
    strip_include_prefix = "/firmware/main",
)

cc_test(
    name = "dribbler_test",
    srcs = ["dribbler_test.cpp"],
    deps = [
        ":dribbler",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "firmware_ball",
    srcs = ["firmware_ball.c"],
    hdrs = ["firmware_ball.h"],
    strip_include_prefix = "/firmware/main",
)

cc_test(
    name = "firmware_ball_test",
    srcs = ["firmware_ball_test.cpp"],
    deps = [
        ":firmware_ball",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "firmware_robot",
    srcs = ["firmware_robot.c"],
    hdrs = ["firmware_robot.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        ":chicker",
        ":dribbler",
        ":wheel",
    ],
)

cc_test(
    name = "firmware_robot_test",
    srcs = ["firmware_robot_test.cpp"],
    deps = [
        ":firmware_robot",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "firmware_world",
    srcs = ["firmware_world.c"],
    hdrs = ["firmware_world.h"],
    strip_include_prefix = "/firmware/main",
    deps = [
        ":firmware_ball",
        ":firmware_robot",
    ],
)

cc_test(
    name = "firmware_world_test",
    srcs = ["firmware_world_test.cpp"],
    deps = [
        ":firmware_world",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "wheel",
    srcs = ["wheel.c"],
    hdrs = ["wheel.h"],
    strip_include_prefix = "/firmware/main",
)

cc_test(
    name = "wheel_test",
    srcs = ["wheel_test.cpp"],
    deps = [
        ":wheel",
        "@gtest//:gtest_main",
    ],
)
