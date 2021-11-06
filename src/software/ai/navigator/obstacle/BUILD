package(default_visibility = ["//software/ai/navigator:__subpackages__"])

cc_library(
    name = "obstacle",
    hdrs = [
        "obstacle.hpp",
    ],
    # We expose obstacle so that it can visualized
    visibility = ["//visibility:public"],
    deps = [
        "obstacle_visitor",
        "//shared:constants",
        "//software/geom:point",
        "//software/geom:segment",
        "//software/geom/algorithms",
    ],
)

cc_library(
    name = "robot_navigation_obstacle_factory",
    srcs = ["robot_navigation_obstacle_factory.cpp"],
    hdrs = ["robot_navigation_obstacle_factory.h"],
    deps = [
        ":obstacle",
        "//shared/parameter:cpp_configs",
        "//software/ai/motion_constraint",
        "//software/geom:point",
        "//software/logger",
        "//software/world",
    ],
)

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cpp"],
    deps = [
        ":obstacle",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:point",
        "//software/geom:rectangle",
    ],
)

cc_test(
    name = "robot_navigation_obstacle_factory_test",
    srcs = ["robot_navigation_obstacle_factory_test.cpp"],
    deps = [
        ":robot_navigation_obstacle_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/geom:point",
        "//software/geom:rectangle",
        "//software/test_util",
        "//software/world",
    ],
)

cc_library(
    name = "obstacle_visitor",
    hdrs = ["obstacle_visitor.h"],
    # We expose obstacle visitor so that it can visualized
    visibility = ["//visibility:public"],
    deps = [
        "//software/geom:circle",
        "//software/geom:polygon",
        "//software/geom:rectangle",
    ],
)
