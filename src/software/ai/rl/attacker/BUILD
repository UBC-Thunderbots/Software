package(default_visibility = ["//visibility:public"])

exports_files(["attacker_mdp_q_function_weights.csv"])

cc_library(
    name = "attacker_mdp_action",
    srcs = ["attacker_mdp_action.cpp"],
    hdrs = ["attacker_mdp_action.h"],
    deps = [
        "//software/ai:strategy",
        "//software/ai/hl/stp/skill:all_skills",
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "attacker_mdp_state",
    hdrs = ["attacker_mdp_state.h"],
    deps = [
        "//software/ai:strategy",
    ],
)

cc_library(
    name = "attacker_mdp_feature_extractor",
    srcs = ["attacker_mdp_feature_extractor.cpp"],
    hdrs = ["attacker_mdp_feature_extractor.h"],
    deps = [
        ":attacker_mdp_action",
        ":attacker_mdp_state",
        "//shared:constants",
        "//software/ai/evaluation:possession",
        "//software/ai/rl:feature_extractor",
        "//software/math:math_functions",
    ],
)

cc_library(
    name = "attacker_mdp_reward_function",
    srcs = ["attacker_mdp_reward_function.cpp"],
    hdrs = ["attacker_mdp_reward_function.h"],
    deps = [
        "//software/ai:strategy",
        "//software/logger",
        "//software/math:math_functions",
        "//software/world",
    ],
)
