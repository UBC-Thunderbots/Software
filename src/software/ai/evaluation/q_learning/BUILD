package(default_visibility = ["//visibility:public"])

cc_library(
    name = "attacker_mdp_action",
    hdrs = ["attacker_mdp_action.h"],
    srcs = ["attacker_mdp_action.cpp"],
    deps = [
        "//software/ai:strategy",
        "//software/ai/hl/stp/skill:all_skills",
        "//software/util/make_enum:reflective_enum",
    ]
)

cc_library(
    name = "attacker_mdp_feature_extractor",
    hdrs = ["attacker_mdp_feature_extractor.h"],
    srcs = ["attacker_mdp_feature_extractor.cpp"],
    deps = [
        ":attacker_mdp_action",
        ":feature_extractor",
        "//software/ai/evaluation:possession",
    ]
)

cc_library(
    name = "feature_extractor",
    hdrs = ["feature_extractor.hpp"],
    deps = [
        "//software/util/make_enum:reflective_enum",
    ]
)

cc_library(
    name = "gameplay_monitor",
    hdrs = ["gameplay_monitor.h"],
    srcs = ["gameplay_monitor.cpp"],
    deps = [
        "//software/logger",
        "//software/world",
    ]
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
        ":q_function",
        ":feature_extractor",
        "@eigen"
    ]
)

cc_library(
    name = "q_policy",
    hdrs = ["q_policy.hpp"],
    deps = [
        ":q_function",
        ":feature_extractor",
        "@eigen"
    ]
)
