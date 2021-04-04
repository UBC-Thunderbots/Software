package(default_visibility = ["//visibility:public"])

cc_library(
    name = "passer_tactic",
    srcs = ["passer_tactic.cpp"],
    hdrs = [
        "passer_fsm.h",
        "passer_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/passing:pass",
        "//software/logger",
        "//software/world:ball",
    ],
)

cc_test(
    name = "passer_fsm_test",
    srcs = ["passer_fsm_test.cpp"],
    deps = [
        ":passer_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_passer_tactic_test",
    srcs = ["simulated_passer_tactic_test.cpp"],
    deps = [
        ":passer_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
