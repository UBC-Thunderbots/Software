package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "pass_skill",
    srcs = [
        "pass_skill_fsm.cpp",
    ],
    hdrs = [
        "pass_skill.h",
        "pass_skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/skill:base_skill",
        "//software/ai/hl/stp/skill/pivot_kick:pivot_kick_skill"
    ],
    alwayslink = True,
)
