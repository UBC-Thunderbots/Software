package(default_visibility = ["//visibility:public"])

cc_library(
    name = "skill",
    hdrs = [
        "skill.h",
        "skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/strategy",
        "//software/ai/hl/stp/tactic:tactic_fsm",
    ],
    alwayslink = True,
)

cc_library(
    name = "all_skills",
    deps = [
        ":shoot_skill",
    ],
)

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
        ":skill",
        "//software/ai/hl/stp/tactic/attacker:attacker_fsm",
        "//software/util/generic_factory:generic_factory",
    ],
    alwayslink = True,
)
