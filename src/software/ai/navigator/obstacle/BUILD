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
    name = "obstacle_factory",
    srcs = ["obstacle_factory.cpp"],
    hdrs = ["obstacle_factory.h"],
    deps = [
        ":obstacle",
        "//software/ai/motion_constraint",
        "//software/new_geom",
        "//software/util/parameter:dynamic_parameters",
        "//software/world",
    ],
)

cc_test(
    name = "obstacle_factory_test",
    srcs = ["obstacle_factory_test.cpp"],
    deps = [
        ":obstacle_factory",
        "@gtest//:gtest_main",
    ],
)
