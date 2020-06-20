package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    hdrs = [
        "obstacle.h",
        "obstacle.tpp",
    ],
    # We expose obstacle so that it can visualized
    visibility = ["//visibility:public"],
    deps = [
        "obstacle_visitor",
        "//shared:constants",
        "//software/new_geom:point",
        "//software/new_geom:segment",
        "//software/new_geom/util:algorithms",
    ],
)

cc_library(
    name = "robot_navigation_obstacle_factory",
    srcs = ["robot_navigation_obstacle_factory.cpp"],
    hdrs = ["robot_navigation_obstacle_factory.h"],
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
    name = "robot_navigation_obstacle_factory_test",
    srcs = ["robot_navigation_obstacle_factory_test.cpp"],
    deps = [
        ":robot_navigation_obstacle_factory",
        "//software/new_geom:point",
        "//software/new_geom:rectangle",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "obstacle_visitor",
    hdrs = ["obstacle_visitor.h"],
    # We expose obstacle visitor so that it can visualized
    visibility = ["//visibility:public"],
    deps = [
        "//software/new_geom:circle",
        "//software/new_geom:polygon",
    ],
)
