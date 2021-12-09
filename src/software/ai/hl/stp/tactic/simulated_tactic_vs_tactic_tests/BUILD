package(default_visibility = ["//visibility:public"])

cc_test(
    name = "simulated_attacker_vs_goalie_tactic_test",
    srcs = ["simulated_attacker_vs_goalie_tactic_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
