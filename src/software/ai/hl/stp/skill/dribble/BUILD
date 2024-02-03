package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "dribble_skill",
    srcs = [
        "dribble_skill_fsm.cpp",
    ],
    hdrs = [
        "dribble_skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/skill",
        "//software/ai/hl/stp/tactic:move_primitive",
    ],
    alwayslink = True,
)
