package(default_visibility = ["//visibility:public"])

cc_library(
    name = "navigator",
    srcs = ["navigator.cpp"],
    hdrs = ["navigator.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":navigating_primitive_creator",
        "//shared/parameter_v2:cpp_configs",
        "//software/ai/intent",
        "//software/ai/intent:all_intents",
        "//software/ai/intent:intent_visitor",
        "//software/ai/navigator/obstacle",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/ai/navigator/path_manager",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/proto/message_translation:tbots_protobuf",
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

cc_library(
    name = "navigating_primitive_creator",
    srcs = ["navigating_primitive_creator.cpp"],
    hdrs = ["navigating_primitive_creator.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared/parameter_v2:cpp_configs",
        "//software/ai/intent:all_intents",
        "//software/ai/intent:navigating_intent",
        "//software/ai/intent:navigating_intent_visitor",
        "//software/ai/navigator/obstacle",
        "//software/ai/navigator/path_planner",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/world",
    ],
)

cc_test(
    name = "navigating_primitive_creator_test",
    srcs = ["navigating_primitive_creator_test.cpp"],
    deps = [
        ":navigating_primitive_creator",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
