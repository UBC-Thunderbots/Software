package(default_visibility = ["//visibility:public"])

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

cc_library(
    name = "hrvo",
    srcs = [
        "agent.cpp",
        "hrvo_agent.cpp",
        "kd_tree.cpp",
        "linear_velocity_agent.cpp",
        "path.cpp",
        "path_point.cpp",
        "simulator.cpp",
    ],
    hdrs = [
        "agent.h",
        "hrvo_agent.h",
        "kd_tree.h",
        "linear_velocity_agent.h",
        "path.h",
        "path_point.h",
        "simulator.h",
    ],
    deps = [
        "//proto:visualization_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/ai/navigator/path_planner/hrvo:velocity_obstacle",
        "//software/geom:vector",
        "//software/networking:threaded_proto_unix_sender",
        "//software/world",
    ],
)
