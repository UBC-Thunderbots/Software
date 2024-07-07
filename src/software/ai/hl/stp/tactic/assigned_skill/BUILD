package(default_visibility = ["//visibility:public"])

cc_library(
    name = "assigned_skill_tactic",
    hdrs = ["assigned_skill_tactic.hpp"],
    deps = [
        "//software/ai/hl/stp/skill:base_skill",
        "//software/ai/hl/stp/tactic",
        "//software/util/is_template_base_of",
    ],
)

cc_library(
    name = "assigned_skill_tactic_copies",
    hdrs = ["assigned_skill_tactic_copies.h"],
    deps = [
        ":assigned_skill_tactic",
        "//software/ai/hl/stp/skill:all_skills",
    ],
)
