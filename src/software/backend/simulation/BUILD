package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_robot",
    srcs = ["simulator_robot.cpp"],
    hdrs = ["simulator_robot.h"],
    deps = [
        "//firmware/main/app/primitives:primitive_manager",
        "//firmware/main/app/world:firmware_world",
        "//software/ai/primitive",
        "//software/backend/simulation/physics:physics_ball",
        "//software/backend/simulation/physics:physics_robot",
        "//software/logger",
    ],
)

cc_test(
    name = "simulator_robot_test",
    srcs = ["simulator_robot_test.cpp"],
    deps = [
        ":simulator_ball",
        ":simulator_robot",
        "//software/backend/simulation/physics:physics_world",
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
        "//software/backend/simulation/physics:physics_ball",
        "//software/logger",
    ],
)

cc_test(
    name = "simulator_ball_test",
    srcs = ["simulator_ball_test.cpp"],
    deps = [
        ":simulator_ball",
        "//software/backend/simulation/physics:physics_ball",
        "//software/backend/simulation/physics:physics_world",
        "//software/test_util",
        "@box2d",
        "@gtest//:gtest_main",
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
        "//firmware/main/shared:physics",
        "//software/logger",
    ],
)

cc_test(
    name = "simulator_robot_singleton_test",
    srcs = ["simulator_robot_singleton_test.cpp"],
    deps = [
        ":simulator_ball",
        ":simulator_robot",
        ":simulator_robot_singleton",
        "//firmware/main/app/world:firmware_robot",
        "//software/backend/simulation/physics:physics_robot",
        "//software/backend/simulation/physics:physics_world",
        "//software/test_util",
        "//software/world",
        "//software/world:robot",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "simulator_ball_singleton",
    srcs = ["simulator_ball_singleton.cpp"],
    hdrs = ["simulator_ball_singleton.h"],
    deps = [
        ":simulator_ball",
        "//firmware/main/app/world:firmware_ball",
        "//software/logger",
        "@box2d",
    ],
)

cc_test(
    name = "simulator_ball_singleton_test",
    srcs = ["simulator_ball_singleton_test.cpp"],
    deps = [
        ":simulator_ball",
        ":simulator_ball_singleton",
        "//firmware/main/app/world:firmware_ball",
        "//software/backend/simulation/physics:physics_world",
        "//software/new_geom:point",
        "//software/new_geom:vector",
        "//software/test_util",
        "//software/time:timestamp",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "simulator",
    srcs = ["simulator.cpp"],
    hdrs = ["simulator.h"],
    deps = [
        ":simulator_ball",
        ":simulator_ball_singleton",
        ":simulator_robot",
        ":simulator_robot_singleton",
        "//firmware/main/app/primitives:primitive_manager",
        "//firmware/main/app/world:firmware_world",
        "//software/ai/primitive",
        "//software/backend/output/radio/mrf:mrf_primitive_visitor",
        "//software/backend/simulation/physics:physics_world",
        "//software/world",
    ],
)

cc_test(
    name = "simulator_test",
    srcs = ["simulator_test.cpp"],
    deps = [
        ":simulator",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
