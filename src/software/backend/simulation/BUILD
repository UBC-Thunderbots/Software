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
        "//software/backend/simulation/physics:physics_simulator",
        "@box2d",
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
