package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cpp"],
    hdrs = ["obstacle.h"],
    deps = [
        "//shared:constants",
        "//software/geom",
        "//software/world:ball",
        "//software/world:robot",
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
    name = "obstacle_generation",
    srcs = ["obstacle_generation.cpp"],
    hdrs = ["obstacle_generation.h"],
    deps = [
        "//software/new_geom",
        ":obstacle",
        "//software/util/parameter:dynamic_parameters",
        "//software/ai/motion_constraint:motion_constraint",
        "//software/world",
    ],
)
