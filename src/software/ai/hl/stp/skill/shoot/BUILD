package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "shoot_skill",
    srcs = [
        "shoot_skill.cpp",
        "shoot_skill_fsm.cpp",
    ],
    hdrs = [
        "shoot_skill.h",
        "shoot_skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/skill:base_skill",
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/ai/hl/stp/skill/pivot_kick:pivot_kick_skill",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)
