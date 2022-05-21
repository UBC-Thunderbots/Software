package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pivot_kick_tactic",
    srcs = [
        "pivot_kick_fsm.cpp",
        "pivot_kick_tactic.cpp",
    ],
    hdrs = [
        "pivot_kick_fsm.h",
        "pivot_kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "pivot_kick_fsm_test",
    srcs = ["pivot_kick_fsm_test.cpp"],
    deps = [
        ":pivot_kick_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "pivot_kick_tactic_test",
    srcs = ["pivot_kick_tactic_test.cpp"],
    deps = [
        ":pivot_kick_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
