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
        "//software/ai/navigator/obstacle:obstacle_generation",
        "//software/ai/navigator/path_manager:path_manager",
        "//software/ai/primitive:all_primitives",
        "//software/util/parameter:dynamic_parameters",
        "//software/world",
        "@g3log",
    ],
)

cc_test(
    name = "navigator_test",
    srcs = ["navigator_test.cpp"],
    deps = [
        ":navigator",
        "//software/test_util",
        "//software/ai/navigator/path_planner:theta_star_path_planner",
        "//software/ai/navigator/path_planner:one_point_path_test_path_planner",
        "//software/ai/navigator/path_planner:no_path_test_path_planner",
        "//software/ai/navigator/path_manager:velocity_obstacle_path_manager",
        "@gtest//:gtest_main",
    ],
)
