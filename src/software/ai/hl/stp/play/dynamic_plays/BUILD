package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "dynamic_play",
    srcs = ["dynamic_play.cpp"],
    hdrs = ["dynamic_play.h"],
    deps = [
        "//software/ai/evaluation/scoring/support_tactics:duplication_scorer",
        "//software/ai/evaluation/scoring/support_tactics:feasibility_scorer",
        "//software/ai/evaluation/scoring/support_tactics:success_scorer",
        "//software/ai/evaluation/scoring/support_tactics:support_tactic_candidate",
        "//software/ai/hl/stp/play",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "offensive_plays",
    hdrs = ["offensive_plays.h"],
    deps = [
        ":dynamic_play",
        "//software/ai/evaluation/scoring/support_tactics:feasibility_scorer",
        "//software/ai/hl/stp/play/defense:defense_play",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
    ],
    alwayslink = True,
)
