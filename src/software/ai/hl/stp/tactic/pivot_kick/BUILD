package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pivot_kick_tactic",
    srcs = ["pivot_kick_tactic.cpp"],
    hdrs = [
        "pivot_kick_fsm.h",
        "pivot_kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:stop_action",
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
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_pivot_kick_tactic_test",
    srcs = ["simulated_pivot_kick_tactic_test.cpp"],
    deps = [
        ":pivot_kick_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
