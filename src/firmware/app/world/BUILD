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
        "//shared/test_util:tbots_gtest_main",
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
        "//shared/test_util:tbots_gtest_main",
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
        "//shared/test_util:tbots_gtest_main",
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
        "//shared/test_util:tbots_gtest_main",
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
        ":force_wheel",
        ":velocity_wheel",
        "//firmware/app/control",
        "//firmware/app/control:physbot",
        "//firmware/app/control:trajectory_planner",
        "//firmware/shared:physics",
        "//firmware/shared:util",
        "//proto:tbots_nanopb_proto",
        "//shared:constants",
    ],
)

cc_test(
    name = "firmware_robot_test",
    srcs = ["firmware_robot_test.cpp"],
    deps = [
        ":firmware_robot",
        "//firmware/app/logger",
        "//shared/test_util",
        "//shared/test_util:tbots_gtest_main",
        "//software/logger",
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
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "force_wheel",
    srcs = ["force_wheel.c"],
    hdrs = ["force_wheel.h"],
    deps = ["//shared:robot_constants"],
)

cc_library(
    name = "velocity_wheel",
    srcs = ["velocity_wheel.c"],
    hdrs = ["velocity_wheel.h"],
    deps = ["//shared:robot_constants"],
)

cc_test(
    name = "force_wheel_test",
    srcs = ["force_wheel_test.cpp"],
    deps = [
        ":force_wheel",
        "//shared/test_util:tbots_gtest_main",
    ],
)
