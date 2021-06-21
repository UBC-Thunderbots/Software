package(default_visibility = ["//visibility:public"])

cc_library(
    name = "receiver_tactic",
    srcs = ["receiver_tactic.cpp"],
    hdrs = [
        "receiver_fsm.h",
        "receiver_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/passing:pass",
        "//software/logger",
        "//software/world:ball",
    ],
)

cc_test(
    name = "receiver_fsm_test",
    srcs = ["receiver_fsm_test.cpp"],
    deps = [
        ":receiver_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_receiver_tactic_test",
    srcs = ["simulated_receiver_tactic_test.cpp"],
    deps = [
        ":receiver_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
