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
    alwayslink = True,
)

cc_library(
    name = "all_skills",
    deps = [
        "//software/ai/hl/stp/skill/keep_away:keep_away_skill",
        "//software/ai/hl/stp/skill/shoot:shoot_skill",
    ],
)
