package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_robot",
    srcs = ["simulator_robot.cpp"],
    hdrs = ["simulator_robot.h"],
    deps = [
        "//firmware/main/app/world:chicker",
        "//firmware/main/app/world:dribbler",
        "//firmware/main/app/world:firmware_robot",
        "//firmware/main/app/world:wheel",
        "//software/backend/simulation/physics:physics_robot",
        "@box2d",
    ],
)

cc_test(
    name = "simulator_robot_test",
    srcs = ["simulator_robot_test.cpp"],
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
    name = "simulator_ball",
    srcs = ["simulator_ball.cpp"],
    hdrs = ["simulator_ball.h"],
    deps = [
        "//firmware/main/app/world:firmware_ball",
        "//software/backend/simulation/physics:physics_ball",
        "@box2d",
    ],
)
