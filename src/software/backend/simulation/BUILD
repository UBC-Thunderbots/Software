package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_robot",
    srcs = ["simulator_robot.cpp"],
    hdrs = ["simulator_robot.h"],
    deps = [
        "//software/ai/primitive",
        "//software/backend/simulation/physics:physics_robot",
    ],
)

cc_library(
    name = "simulator_robot_singleton",
    srcs = ["simulator_robot_singleton.cpp"],
    hdrs = ["simulator_robot_singleton.h"],
    deps = [
        ":simulator_robot",
        "//firmware/main/app/world:chicker",
        "//firmware/main/app/world:dribbler",
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/app/world:wheel",
    ],
)

cc_test(
    name = "simulator_robot_singleton_test",
    srcs = ["simulator_robot_singleton_test.cpp"],
    deps = [
        ":simulator_robot",
        "//firmware/main/app/world:firmware_robot",
        "//software/backend/simulation/physics:physics_robot",
        "//software/backend/simulation/physics:physics_simulator",
        "//software/test_util",
        "//software/world",
        "//software/world:robot",
        "@box2d",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "simulator_ball_singleton",
    srcs = ["simulator_ball_singleton.cpp"],
    hdrs = ["simulator_ball_singleton.h"],
    deps = [
        "//firmware/main/app/world:firmware_ball",
        "//software/backend/simulation/physics:physics_ball",
        "@box2d",
    ],
)

cc_library(
    name = "simulator",
    srcs = ["simulator.cpp"],
    hdrs = ["simulator.h"],
    deps = [
        "//software/ai/primitive",
        "//software/world",
    ],
)
