package(default_visibility = ["//visibility:public"])

cc_library(
    name = "penalty_kick_tactic",
    srcs = [
        "penalty_kick_fsm.cpp",
        "penalty_kick_tactic.cpp",
    ],
    hdrs = [
        "penalty_kick_fsm.h",
        "penalty_kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "penalty_kick_fsm_test",
    srcs = ["penalty_kick_fsm_test.cpp"],
    deps = [
        ":penalty_kick_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "penalty_kick_tactic_test",
    srcs = ["penalty_kick_tactic_test.cpp"],
    deps = [
        ":penalty_kick_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
