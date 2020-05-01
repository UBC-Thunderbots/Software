package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    srcs = [
        "circle_obstacle.cpp",
        "obstacle.cpp",
        "polygon_obstacle.cpp",
    ],
    hdrs = [
        "circle_obstacle.h",
        "obstacle.h",
        "polygon_obstacle.h",
    ],
    deps = [
        "//shared:constants",
        "//software/new_geom:circle",
        "//software/new_geom:point",
        "//software/new_geom:polygon",
        "//software/new_geom:segment",
        "//software/new_geom/util",
    ],
)

cc_library(
    name = "obstacle_factory",
    srcs = ["obstacle_factory.cpp"],
    hdrs = ["obstacle_factory.h"],
    deps = [
        ":obstacle",
        "//software/ai/motion_constraint",
        "//software/geom",
        "//software/logger",
        "//software/new_geom:point",
        "//software/parameter:dynamic_parameters",
        "//software/world",
    ],
)

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cpp"],
    deps = [
        ":obstacle",
        "//software/new_geom:point",
        "//software/new_geom:rectangle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "obstacle_factory_test",
    srcs = ["obstacle_factory_test.cpp"],
    deps = [
        ":obstacle_factory",
        "//software/new_geom:point",
        "//software/new_geom:rectangle",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
