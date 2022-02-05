package(default_visibility = ["//visibility:public"])

cc_test(
    name = "friendly_vs_enemy_penalty_kick_play_test",
    srcs = ["friendly_vs_enemy_penalty_kick_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/play:penalty_kick_enemy_play",
        "//software/ai/hl/stp/play:penalty_kick_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
