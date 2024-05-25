package(default_visibility = ["//visibility:public"])

cc_library(
    name = "attacker_tactic",
    srcs = ["attacker_tactic.cpp"],
    hdrs = ["attacker_tactic.h"],
    deps = [
        "//shared:constants",
        "//software/ai:strategy",
        "//software/ai/evaluation/q_learning:attacker_mdp_action",
        "//software/ai/evaluation/q_learning:attacker_mdp_feature_extractor",
        "//software/ai/evaluation/q_learning:gameplay_monitor",
        "//software/ai/evaluation/q_learning:linear_q_function",
        "//software/ai/evaluation/q_learning:q_policy",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "attacker_tactic_passing_test",
    srcs = ["attacker_tactic_passing_test.cpp"],
    deps = [
        ":attacker_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "attacker_tactic_shoot_goal_test",
    srcs = ["attacker_tactic_shoot_goal_test.cpp"],
    deps = [
        ":attacker_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "attacker_tactic_keep_away_test",
    srcs = ["attacker_tactic_keep_away_test.cpp"],
    deps = [
        ":attacker_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
