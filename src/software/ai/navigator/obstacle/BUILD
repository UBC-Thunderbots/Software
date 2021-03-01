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
        "//shared/parameter_v2:cpp_configs",
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
        "//software/geom:point",
        "//software/geom:rectangle",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "robot_navigation_obstacle_factory_test",
    srcs = ["robot_navigation_obstacle_factory_test.cpp"],
    deps = [
        ":robot_navigation_obstacle_factory",
        "//software/geom:point",
        "//software/geom:rectangle",
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
        "//software/geom:circle",
        "//software/geom:polygon",
    ],
)
