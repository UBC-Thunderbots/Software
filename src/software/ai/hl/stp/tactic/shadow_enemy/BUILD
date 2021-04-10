package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shadow_enemy_tactic",
    srcs = ["shadow_enemy_tactic.cpp"],
    hdrs = [
        "shadow_enemy_fsm.h",
        "shadow_enemy_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/intent:move_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "shadow_enemy_fsm_test",
    srcs = ["shadow_enemy_fsm_test.cpp"],
    deps = [
        ":shadow_enemy_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_shadow_enemy_tactic_test",
    srcs = ["simulated_shadow_enemy_tactic_test.cpp"],
    deps = [
        ":shadow_enemy_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
