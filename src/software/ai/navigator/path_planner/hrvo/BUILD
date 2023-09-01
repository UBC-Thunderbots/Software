package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velocity_obstacle",
    srcs = ["velocity_obstacle.cpp"],
    hdrs = [
        "velocity_obstacle.h",
    ],
    deps = [
        "//software/geom:circle",
        "//software/geom:vector",
        "//software/logger",
    ],
)

cc_library(
    name = "hrvo",
    srcs = [
        "agent.cpp",
        "hrvo_agent.cpp",
        "hrvo_simulator.cpp",
        "lv_agent.cpp",
        "path_point.cpp",
        "robot_path.cpp",
    ],
    hdrs = [
        "agent.h",
        "hrvo_agent.h",
        "hrvo_simulator.h",
        "lv_agent.h",
        "path_point.h",
        "robot_path.h",
    ],
    deps = [
        "//proto:visualization_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/ai/navigator/path_planner/hrvo:velocity_obstacle",
        "//software/geom:vector",
        "//software/geom/algorithms:nearest_neighbor_search",
        "//software/networking:threaded_proto_unix_sender",
        "//software/physics:velocity_conversion_util",
        "//software/world",
        "//software/ai/navigator/path_planner:bang_bang_trajectory_2d",
        "//software/ai/navigator/path_planner:bang_bang_trajectory_1d_angular",
    ],
)

cc_test(
    name = "velocity_obstacle_test",
    srcs = ["velocity_obstacle_test.cpp"],
    deps = [
        "velocity_obstacle",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "hrvo_test",
    srcs = ["hrvo_test.cpp"],
    deps = [
        ":hrvo",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "simulated_hrvo_test",
    srcs = ["simulated_hrvo_test.cpp"],
    deps = [
        ":hrvo",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
