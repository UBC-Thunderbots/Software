package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_tactic",
    srcs = ["move_tactic.cpp"],
    hdrs = [
        "move_fsm.h",
        "move_tactic.h",
    ],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/intent:move_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "move_fsm_test",
    srcs = ["move_fsm_test.cpp"],
    deps = [
        ":move_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "move_tactic_test",
    srcs = ["move_tactic_test.cpp"],
    deps = [
        ":move_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_move_tactic_test",
    srcs = ["simulated_move_tactic_test.cpp"],
    deps = [
        ":move_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/validation:validation_function",
        "//software/simulated_tests/validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

