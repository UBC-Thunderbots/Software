package(default_visibility = ["//visibility:public"])

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
    ],
)
