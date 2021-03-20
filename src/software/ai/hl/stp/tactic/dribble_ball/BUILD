package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dribble_ball_tactic",
    srcs = ["dribble_ball_tactic.cpp"],
    hdrs = [
        "dribble_ball_fsm.h",
        "dribble_ball_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/get_possession:get_possession_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/intent:move_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "dribble_ball_fsm_test",
    srcs = ["dribble_ball_fsm_test.cpp"],
    deps = [
        ":dribble_ball_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_dribble_ball_tactic_test",
    srcs = ["simulated_dribble_ball_tactic_test.cpp"],
    deps = [
        ":dribble_ball_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
