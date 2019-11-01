package(default_visibility = ["//software/ai:__subpackages__"])

cc_library(
    name = "path_manager",
    hdrs = ["path_manager.h",
            "path_objective.h",
    ],
    deps = [
        "//software/ai/navigator/obstacle",
        "//software/geom",
        "//software/ai/navigator/path_planner:path_planner",
    ],
)

cc_library(
    name = "obstacle_path_manager",
    srcs = ["obstacle_path_manager.cpp"],
    hdrs = ["obstacle_path_manager.h"],
    deps = [
        ":path_manager",
        "//software/util/parameter:dynamic_parameters",
    ],
)

cc_test(
    name = "obstacle_path_manager_test",
    srcs = ["obstacle_path_manager_test.cpp"],
    deps = [
        ":obstacle_path_manager",
        "//software/test_util",
        "//software/ai/navigator/path_planner:straight_line_path_planner",
        "@gtest//:gtest_main",
    ],
)
