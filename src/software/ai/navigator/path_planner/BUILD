package(default_visibility = [
    "//software/ai:__subpackages__",
    "//software/simulated_tests:__subpackages__",
])

cc_library(
    name = "enlsvg_path_planner",
    srcs = ["enlsvg_path_planner.cpp"],
    hdrs = ["enlsvg_path_planner.h"],
    deps = [
        "//extlibs/enlsvg",
        "//software/ai/navigator/obstacle",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/geom:linear_spline2d",
        "//software/logger",
    ],
)

cc_library(
    name = "global_path_planner_factory",
    srcs = ["global_path_planner_factory.cpp"],
    hdrs = ["global_path_planner_factory.h"],
    deps = [
        ":enlsvg_path_planner",
        "//software/ai/hl/stp/tactic",
        "//software/ai/motion_constraint:motion_constraint_set_builder",
        "//software/geom:linear_spline2d",
        "//software/geom:point",
    ],
)

cc_library(
    name = "itrajectory",
    hdrs = ["ITrajectory.h"],
    deps = [
        "//software/time:duration",
    ],
)

cc_test(
    name = "enlsvg_path_planner_test",
    srcs = ["enlsvg_path_planner_test.cpp"],
    deps = [
        ":enlsvg_path_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util",
    ],
)

cc_test(
    name = "global_path_planner_factory_test",
    srcs = ["global_path_planner_factory_test.cpp"],
    deps = [
        ":global_path_planner_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)
