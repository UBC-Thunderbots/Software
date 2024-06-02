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
        "//software/ai/evaluation/q_learning/bandits:epsilon_greedy_strategy",
        "//software/ai/evaluation/q_learning/bandits:softmax_strategy",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/tactic",
        "//software/logger",
    ],
)
