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
        "shape_obstacle.h",
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

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cpp"],
    deps = [
        ":obstacle",
        "//software/new_geom:point",
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
        "//software/geom",
        "//software/new_geom:point",
        "//software/parameter:dynamic_parameters",
        "//software/world",
    ],
)
