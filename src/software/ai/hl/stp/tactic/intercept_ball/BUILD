package(default_visibility = ["//visibility:public"])

cc_library(
    name = "intercept_ball_tactic",
    srcs = ["intercept_ball_tactic.cpp"],
    hdrs = [
        "intercept_ball_fsm.h",
        "intercept_ball_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/intent:move_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "intercept_ball_fsm_test",
    srcs = ["intercept_ball_fsm_test.cpp"],
    deps = [
        ":intercept_ball_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_intercept_ball_tactic_test",
    srcs = ["simulated_intercept_ball_tactic_test.cpp"],
    deps = [
        ":intercept_ball_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/validation:validation_function",
        "//software/simulated_tests/validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
