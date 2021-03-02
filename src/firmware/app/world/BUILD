package(default_visibility = ["//visibility:public"])

cc_library(
    name = "charger",
    srcs = ["charger.c"],
    hdrs = ["charger.h"],
)

cc_test(
    name = "charger_test",
    srcs = ["charger_test.cpp"],
    deps = [
        ":charger",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "chicker",
    srcs = ["chicker.c"],
    hdrs = ["chicker.h"],
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
    deps = [
        ":charger",
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
        "//firmware/app/logger",
        "//shared/test_util",
        "//software/logger",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "firmware_world",
    srcs = ["firmware_world.c"],
    hdrs = ["firmware_world.h"],
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
)

cc_test(
    name = "wheel_test",
    srcs = ["wheel_test.cpp"],
    deps = [
        ":wheel",
        "@gtest//:gtest_main",
    ],
)
