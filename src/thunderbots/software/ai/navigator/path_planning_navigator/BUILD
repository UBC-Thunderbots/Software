package(default_visibility = ["//visibility:private"])

cc_library(
    name = "path_planning_navigator",
    srcs = ["path_planning_navigator.cpp"],
    hdrs = ["path_planning_navigator.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":util",
        "//software/ai/intent",
        "//software/ai/intent:all_intents",
        "//software/ai/intent:intent_visitor",
        "//software/ai/navigator",
        "//software/ai/navigator/path_planning_navigator/obstacle",
        "//software/ai/navigator/path_planning_navigator/path_planner:straight_line_path_planner",
        "//software/ai/navigator/path_planning_navigator/path_planner:theta_star_path_planner",
        "//software/ai/primitive:all_primitives",
        "//software/ai/world",
        "//software/util/canvas_messenger",
        "//software/util/parameter:dynamic_parameters",
    ],
)

cc_test(
    name = "path_planning_navigator_test",
    srcs = ["path_planning_navigator_test.cpp"],
    deps = [
        ":path_planning_navigator",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cpp"],
    hdrs = ["util.h"],
    deps = [
        "//software/ai/primitive:move_primitive",
        "//software/geom:all",
    ],
)

cc_test(
    name = "util_test",
    srcs = ["util_test.cpp"],
    deps = [
        ":util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "trespass",
    srcs = ["trespass.cpp"],
    hdrs = ["trespass.h"],
    deps = [
        "//software/geom:all",
        "//software/util/math:math_functions",
    ],
)

cc_test(
    name = "trespass_test",
    srcs = ["trespass_test.cpp"],
    deps = [
        ":trespass",
        "@gtest//:gtest_main",
    ],
)
