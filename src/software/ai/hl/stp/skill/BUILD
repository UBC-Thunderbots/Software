package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    hdrs = [
        "skill.h",
        "skill_fsm.h",
    ],
    deps = [
        "//proto/primitive:primitive_msg_factory",
        "//software/ai/evaluation/scoring/skills:skill_visitor",
        "//software/ai/hl/stp/primitive",
        "//software/ai/hl/stp/primitive:stop_primitive",
        "//software/ai/hl/stp/strategy",
        "//software/util/sml_fsm",
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
        "//software/ai/hl/stp/skill/get_behind_ball:get_behind_ball_skill",
        "//software/ai/hl/stp/skill/keep_away:keep_away_skill",
        "//software/ai/hl/stp/skill/kick:kick_skill",
        "//software/ai/hl/stp/skill/pass:pass_skill",
        "//software/ai/hl/stp/skill/shoot:shoot_skill",
    ],
)
