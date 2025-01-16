package(default_visibility = ["//visibility:public"])

cc_library(
    name = "attacker_tactic",
    srcs = ["attacker_tactic.cpp"],
    hdrs = ["attacker_tactic.h"],
    deps = [
        ":attacker_skill_executor",
        ":attacker_state",
        "//software/ai/hl/stp/tactic",
        "//software/ai/rl:dqn",
        "//software/ai/rl/exploration_strategies:epsilon_greedy_strategy",
        "//software/ai/rl/replay:prioritized_replay_buffer",
    ],
)

cc_library(
    name = "attacker_state",
    hdrs = ["attacker_state.h"],
    deps = [
        "//software/ai/rl:torch",
    ],
)

cc_library(
    name = "attacker_skill_executor",
    srcs = ["attacker_skill_executor.cpp"],
    hdrs = ["attacker_skill_executor.h"],
    deps = [
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:shot",
        "//software/ai/hl/stp/tactic/pivot_kick:pivot_kick_tactic",
        "//software/ai/passing:pass",
        "//software/util/make_enum",
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
