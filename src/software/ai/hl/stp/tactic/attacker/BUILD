package(default_visibility = ["//visibility:public"])

cc_library(
    name = "attacker_tactic",
    srcs = ["attacker_tactic.cpp"],
    hdrs = ["attacker_tactic.h"],
    data = [
        "//software/ai/rl/attacker:attacker_mdp_q_function_weights.csv",
    ],
    deps = [
        "//shared:constants",
        "//software/ai:strategy",
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/tactic",
        "//software/ai/rl:linear_q_function",
        "//software/ai/rl:q_policy",
        "//software/ai/rl/action_selection:epsilon_greedy_strategy",
        "//software/ai/rl/action_selection:softmax_strategy",
        "//software/ai/rl/attacker:attacker_mdp_action",
        "//software/ai/rl/attacker:attacker_mdp_feature_extractor",
        "//software/ai/rl/attacker:attacker_mdp_reward_function",
        "//software/logger",
    ],
)
