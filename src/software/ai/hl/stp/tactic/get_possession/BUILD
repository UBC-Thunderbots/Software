package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_possession_tactic",
    srcs = ["get_possession_tactic.cpp"],
    hdrs = [
        "get_possession_fsm.h",
        "get_possession_tactic.h",
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
    name = "get_possession_fsm_test",
    srcs = ["get_possession_fsm_test.cpp"],
    deps = [
        ":get_possession_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_get_possession_tactic_test",
    srcs = ["simulated_get_possession_tactic_test.cpp"],
    deps = [
        ":get_possession_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
