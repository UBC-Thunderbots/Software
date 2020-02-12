package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cpp"],
    hdrs = ["obstacle.h"],
    deps = [
        "//shared:constants",
        "//software/geom",
        "//software/new_geom:angle",
        "//software/new_geom:point",
        "//software/new_geom/util:distance",
        "//software/new_geom/util:intersects",
        "//software/world:ball",
        "//software/world:robot",
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
