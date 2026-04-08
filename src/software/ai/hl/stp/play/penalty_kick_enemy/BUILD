package(default_visibility = ["//visibility:public"])

cc_library(
    name = "penalty_kick_enemy_play",
    srcs = [
        "penalty_kick_enemy_play.cpp",
        "penalty_kick_enemy_play_fsm.cpp",
    ],
    hdrs = [
        "penalty_kick_enemy_play.h",
        "penalty_kick_enemy_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
    ],
    alwayslink = True,
)

cc_test(
    name = "penalty_kick_enemy_play_fsm_test",
    srcs = ["penalty_kick_enemy_play_fsm_test.cpp"],
    deps = [
        ":penalty_kick_enemy_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
