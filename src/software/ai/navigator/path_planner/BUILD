package(default_visibility = [
    "//software/ai:__subpackages__",
    "//software/simulated_tests:__subpackages__",
])

cc_library(
    name = "path_planner",
    hdrs = ["path_planner.h"],
    deps = [
        "//software/ai/navigator/obstacle",
        "//software/geom:linear_spline2d",
    ],
)

cc_library(
    name = "straight_line_path_planner",
    srcs = ["straight_line_path_planner.cpp"],
    hdrs = ["straight_line_path_planner.h"],
    deps = [":path_planner"],
)

cc_test(
    name = "straight_line_path_planner_test",
    srcs = ["straight_line_path_planner_test.cpp"],
    deps = [
        ":straight_line_path_planner",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "theta_star_path_planner",
    srcs = ["theta_star_path_planner.cpp"],
    hdrs = ["theta_star_path_planner.h"],
    deps = [
        ":path_planner",
        "//software/geom/algorithms",
    ],
)

cc_library(
    name = "no_path_test_path_planner",
    srcs = ["no_path_test_path_planner.cpp"],
    hdrs = ["no_path_test_path_planner.h"],
    deps = [
        ":path_planner",
    ],
)

cc_library(
    name = "one_point_path_test_path_planner",
    srcs = ["one_point_path_test_path_planner.cpp"],
    hdrs = ["one_point_path_test_path_planner.h"],
    deps = [
        ":path_planner",
    ],
)

cc_library(
    name = "enlsvg_path_planner",
    srcs = ["enlsvg_path_planner.cpp"],
    hdrs = ["enlsvg_path_planner.h"],
    deps = [
        ":path_planner",
        "//extlibs/enlsvg:enlsvg",
        "//software/world:world",
    ],
)

cc_test(
    name = "theta_star_path_planner_test",
    srcs = ["theta_star_path_planner_test.cpp"],
    deps = [
        ":theta_star_path_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util",
        "//software/world:field",
    ],
)

cc_test(
    name = "enlsvg_path_planner_test",
    srcs = ["enlsvg_path_planner_test.cpp"],
    deps = [
        ":enlsvg_path_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util"
    ]
)

cc_test(
    name = "simulated_theta_star_test",
    srcs = ["simulated_theta_star_test.cpp"],
    deps = [
        ":theta_star_path_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "path_planner_test",
    srcs = ["path_planner_test.cpp"],
    deps = [
        ":path_planner",
        ":straight_line_path_planner",
        ":theta_star_path_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util",
        "//software/util/typename",
    ],
)

cc_test(
    name = "path_planner_performance_test",
    srcs = ["path_planner_performance_test.cpp"],
    deps = [
        ":path_planner",
        ":straight_line_path_planner",
        ":theta_star_path_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util",
        "//software/util/typename",
    ],
)
