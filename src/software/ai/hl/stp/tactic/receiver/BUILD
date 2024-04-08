package(default_visibility = ["//visibility:public"])

cc_library(
    name = "receiver_tactic",
    srcs = [
        "receiver_fsm.cpp",
        "receiver_tactic.cpp",
    ],
    hdrs = [
        "receiver_fsm.h",
        "receiver_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/ai/hl/stp/tactic",
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
    name = "receiver_tactic_test",
    srcs = ["receiver_tactic_test.cpp"],
    deps = [
        ":receiver_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
