package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kick_or_chip_tactic",
    srcs = [
        "kick_or_chip_fsm.cpp",
        "kick_or_chip_tactic.cpp",
    ],
    hdrs = [
        "kick_or_chip_fsm.h",
        "kick_or_chip_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "kick_or_chip_fsm_test",
    srcs = ["kick_or_chip_fsm_test.cpp"],
    deps = [
        ":kick_or_chip_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

