package(default_visibility = ["//visibility:public"])

exports_files(["attacker_mdp_q_function_weights.csv"])

cc_library(
    name = "attacker_mdp_action",
    srcs = ["attacker_mdp_action.cpp"],
    hdrs = ["attacker_mdp_action.h"],
    deps = [
        "//software/ai:strategy",
        "//software/ai/hl/stp/skill:all_skills",
        "//software/util/make_enum:reflective_enum",
    ],
)

cc_library(
    name = "attacker_mdp_feature_extractor",
    srcs = ["attacker_mdp_feature_extractor.cpp"],
    hdrs = ["attacker_mdp_feature_extractor.h"],
    deps = [
        ":attacker_mdp_action",
        ":feature_extractor",
        "//software/ai/evaluation:possession",
    ],
)

cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.hpp"],
    deps = [
        "//software/util/make_enum:reflective_enum",
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

cc_library(
    name = "q_function",
    hdrs = ["q_function.hpp"],
    deps = [
        "//software/util/make_enum:reflective_enum",
    ],
)

cc_library(
    name = "linear_q_function",
    hdrs = ["linear_q_function.hpp"],
    deps = [
        ":feature_extractor",
        ":q_function",
        "@csv",
        "@eigen",
    ],
)

cc_library(
    name = "q_policy",
    hdrs = ["q_policy.hpp"],
    deps = [
        ":feature_extractor",
        ":q_function",
        "//software/ai/evaluation/q_learning/bandits:action_selection_strategy",
        "@eigen",
    ],
)
