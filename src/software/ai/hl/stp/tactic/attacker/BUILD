package(default_visibility = ["//visibility:public"])

cc_library(
    name = "attacker_tactic",
    srcs = [
        "attacker_fsm.cpp",
        "attacker_tactic.cpp",
    ],
    hdrs = [
        "attacker_fsm.h",
        "attacker_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:keep_away",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/keep_away:keep_away_tactic",
        "//software/ai/hl/stp/tactic/pivot_kick:pivot_kick_tactic",
        "//software/ai/passing:pass",
        "//software/logger",
        "//software/world:ball",
    ],
)

cc_test(
    name = "attacker_fsm_test",
    srcs = ["attacker_fsm_test.cpp"],
    deps = [
        ":attacker_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
