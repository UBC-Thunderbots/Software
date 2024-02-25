package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "keep_away_skill",
    srcs = [
        "keep_away_skill.cpp",
        "keep_away_skill_fsm.cpp",
    ],
    hdrs = [
        "keep_away_skill.h",
        "keep_away_skill_fsm.h",
    ],
    deps = [
        "//software/ai/evaluation:keep_away",
        "//software/ai/hl/stp/skill:base_skill",
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)
