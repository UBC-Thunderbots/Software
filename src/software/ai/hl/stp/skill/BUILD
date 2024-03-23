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
        "//software/ai/hl/stp/strategy:strategy",
        "//software/ai/hl/stp/primitive:primitive",
        "//software/ai/hl/stp/primitive:stop_primitive",
        "//software/util/sml_fsm:sml_fsm",
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
        "//software/ai/hl/stp/skill/keep_away:keep_away_skill",
        "//software/ai/hl/stp/skill/pass:pass_skill",
        "//software/ai/hl/stp/skill/shoot:shoot_skill",
    ],
)
