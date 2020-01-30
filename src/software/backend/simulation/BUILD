package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_robot_singleton",
    srcs = ["simulator_robot_singleton.cpp"],
    hdrs = ["simulator_robot_singleton.h"],
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
