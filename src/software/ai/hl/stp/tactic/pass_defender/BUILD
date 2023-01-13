package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pass_defender_tactic",
    srcs = [
        "pass_defender_fsm.cpp",
        "pass_defender_tactic.cpp",
    ],
    hdrs = [
        "pass_defender_fsm.h",
        "pass_defender_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)
