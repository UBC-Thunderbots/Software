package(default_visibility = ["//visibility:public"])

cc_library(
    name = "assigned_skill_tactic",
    hdrs = ["assigned_skill_tactic.hpp"],
    deps = [
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/strategy",
        "//software/ai/hl/stp/tactic",
    ],
)

cc_library(
    name = "assigned_skill_tactics",
    hdrs = ["assigned_skill_tactics.h"],
    deps = [
        ":assigned_skill_tactic",
        "//software/ai/hl/stp/skill/pivot_kick:pivot_kick_skill",
    ],
)
