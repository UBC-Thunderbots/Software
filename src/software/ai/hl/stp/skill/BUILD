package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    hdrs = [
        "skill.h",
        "skill_fsm.h",
    ],
    deps = [
        ":skill_state",
        "//proto/primitive:primitive_msg_factory",
        "//software/ai:strategy",
        "//software/ai/hl/stp/primitive",
        "//software/ai/hl/stp/primitive:stop_primitive",
        "//software/util/sml_fsm",
    ],
)

cc_library(
    name = "base_skill",
    hdrs = ["base_skill.hpp"],
    deps = [
        ":skill",
        "//software/ai/hl/stp/tactic:tactic",
    ],
)

cc_library(
    name = "skill_state",
    hdrs = ["skill_state.h"],
    deps = [        
        "//software/ai/passing:pass",
        "//software/ai/evaluation:shot",
    ]
)

cc_library(
    name = "all_skills",
    deps = [
        "//software/ai/hl/stp/skill/chip:chip_skill",
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/ai/hl/stp/skill/get_behind_ball:get_behind_ball_skill",
        "//software/ai/hl/stp/skill/keep_away:keep_away_skill",
        "//software/ai/hl/stp/skill/kick:kick_skill",
        "//software/ai/hl/stp/skill/pass:pass_skill",
        "//software/ai/hl/stp/skill/shoot:shoot_skill",
    ],
)
