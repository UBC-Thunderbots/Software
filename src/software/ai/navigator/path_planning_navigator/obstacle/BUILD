package(default_visibility = ["//software/ai/navigator/path_planning_navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cpp"],
    hdrs = ["obstacle.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world:ball",
        "//software/ai/world:robot",
        "//software/geom",
    ],
)

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cpp"],
    deps = [
        ":obstacle",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "robot_obstacle",
    srcs = ["robot_obstacle.cpp"],
    hdrs = ["robot_obstacle.h"],
    deps = [
        "//shared:constants",
        "//software/ai/world",
        "//software/geom",
        "//software/util/parameter:dynamic_parameters",
    ],
)
