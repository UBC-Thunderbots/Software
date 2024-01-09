package(default_visibility = [
    "//software/ai:__subpackages__",
    "//software/simulated_tests:__subpackages__",
])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "kinematic_constraints",
    hdrs = ["kinematic_constraints.h"],
)

cc_library(
    name = "trajectory",
    hdrs = ["trajectory.hpp"],
)

cc_library(
    name = "trajectory_2d",
    hdrs = ["trajectory_2d.h"],
    deps = [
        ":trajectory",
        "//software/geom:point",
    ],
)

cc_library(
    name = "bang_bang_trajectory_1d",
    srcs = ["bang_bang_trajectory_1d.cpp"],
    hdrs = ["bang_bang_trajectory_1d.h"],
    deps = [
        ":trajectory",
        "//software/geom/algorithms:is_in_range",
        "//software/logger",
    ],
)

cc_library(
    name = "bang_bang_trajectory_1d_angular",
    srcs = ["bang_bang_trajectory_1d_angular.cpp"],
    hdrs = ["bang_bang_trajectory_1d_angular.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":bang_bang_trajectory_1d",
        "//software/geom:angle",
        "//software/geom:angular_acceleration",
        "//software/geom:angular_velocity",
    ],
)

cc_library(
    name = "bang_bang_trajectory_2d",
    srcs = ["bang_bang_trajectory_2d.cpp"],
    hdrs = ["bang_bang_trajectory_2d.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":bang_bang_trajectory_1d",
        ":trajectory_2d",
        "//software/ai/navigator/trajectory:kinematic_constraints",
        "//software/geom:point",
        "//software/geom:rectangle",
    ],
)

cc_library(
    name = "trajectory_path_node",
    hdrs = ["trajectory_path_node.h"],
    deps = [
        ":bang_bang_trajectory_2d",
        "//software/geom:point",
    ],
)

cc_library(
    name = "trajectory_path",
    srcs = ["trajectory_path.cpp"],
    hdrs = ["trajectory_path.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":trajectory_path_node",
        "//software/ai/navigator/trajectory:kinematic_constraints",
        "//software/logger",
    ],
)

cc_library(
    name = "trajectory_path_with_cost",
    srcs = ["trajectory_path_with_cost.cpp"],
    hdrs = ["trajectory_path_with_cost.h"],
    deps = [
        ":trajectory_path",
        "//software/ai/navigator/obstacle",
    ],
)

cc_library(
    name = "trajectory_planner",
    srcs = ["trajectory_planner.cpp"],
    hdrs = ["trajectory_planner.h"],
    deps = [
        ":kinematic_constraints",
        ":trajectory_path",
        "//proto/message_translation:tbots_protobuf",
        "//software/ai/navigator/obstacle",
        "//software/ai/navigator/trajectory:trajectory_path_with_cost",
        "@aabbcc",
        "@tracy",  # TODO (NIMA): Remove
    ],
)

cc_test(
    name = "bang_bang_trajectory_1d_test",
    srcs = ["bang_bang_trajectory_1d_test.cpp"],
    deps = [
        ":bang_bang_trajectory_1d",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "bang_bang_trajectory_2d_test",
    srcs = ["bang_bang_trajectory_2d_test.cpp"],
    deps = [
        ":bang_bang_trajectory_2d",
        "//shared/test_util:tbots_gtest_main",
        "//software/logger",
        "//software/test_util",
    ],
)

cc_test(
    name = "trajectory_planner_test",
    srcs = ["trajectory_planner_test.cpp"],
    deps = [
        ":trajectory_planner",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/test_util",
    ],
)

py_test(
    name = "simulated_hrvo_test",
    srcs = ["simulated_hrvo_test.py"],
    tags = ["exclusive"],
    deps = [
        "//proto/message_translation:py_tbots_protobuf",
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
