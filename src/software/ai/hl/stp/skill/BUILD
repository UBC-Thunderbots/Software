package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    hdrs = [
        "skill.h",
        "skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/strategy",
    ],
)

cc_library(
    name = "base_skill",
    hdrs = ["base_skill.hpp"],
    deps = [":skill"],
)

cc_library(
    name = "all_skills",
    deps = [
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/ai/hl/stp/skill/keep_away:keep_away_skill",
        "//software/ai/hl/stp/skill/pivot_kick:pivot_kick_skill",
        "//software/ai/hl/stp/skill/shoot:shoot_skill",
    ],
)
