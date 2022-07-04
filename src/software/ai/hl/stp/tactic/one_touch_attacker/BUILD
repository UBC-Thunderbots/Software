package(default_visibility = ["//visibility:public"])

cc_library(
    name = "one_touch_attacker_tactic",
    srcs = [
        "one_touch_attacker_fsm.cpp",
        "one_touch_attacker_tactic.cpp",
    ],
    hdrs = [
        "one_touch_attacker_fsm.h",
        "one_touch_attacker_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/passing:pass",
        "//software/ai/evaluation:calc_best_shot",
        "//software/logger",
        "//software/world:ball",
    ],
)
