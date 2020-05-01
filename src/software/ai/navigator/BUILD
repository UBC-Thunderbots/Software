package(default_visibility = ["//visibility:public"])

cc_library(
    name = "navigator",
    srcs = ["navigator.cpp"],
    hdrs = ["navigator.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//software/ai/intent",
        "//software/ai/intent:all_intents",
        "//software/ai/intent:intent_visitor",
        "//software/ai/navigator/obstacle",
        "//software/ai/navigator/obstacle:obstacle_factory",
        "//software/ai/navigator/path_manager",
        "//software/logger",
        "//software/new_geom/util:distance",
        "//software/parameter:dynamic_parameters",
        "//software/primitive:all_primitives",
        "//software/world",
    ],
)

cc_test(
    name = "navigator_test",
    srcs = ["navigator_test.cpp"],
    deps = [
        ":navigator",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "//software/ai/navigator/path_planner:no_path_test_path_planner",
        "//software/ai/navigator/path_planner:one_point_path_test_path_planner",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
