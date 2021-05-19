package(default_visibility = ["//visibility:public"])

cc_library(
    name = "penalty_kick_tactic",
    srcs = ["penalty_kick_tactic.cpp"],
    hdrs = [
        "penalty_kick_fsm.h",
        "penalty_kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
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
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_penalty_kick_tactic_test",
    srcs = ["simulated_penalty_kick_tactic_test.cpp"],
    deps = [
        ":penalty_kick_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
