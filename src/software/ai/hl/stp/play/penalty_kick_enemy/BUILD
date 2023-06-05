package(default_visibility = ["//visibility:public"])

cc_library(
    name = "penalty_kick_enemy_play",
    srcs = [
        "penalty_kick_enemy_play.cpp",
        "penalty_kick_enemy_play_fsm.cpp"
    ],
    hdrs = [
        "penalty_kick_enemy_play.h",
        "penalty_kick_enemy_play_fsm.h"
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play:play",
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

cc_test(
    name = "penalty_kick_enemy_play_test",
    srcs = ["penalty_kick_enemy_play_test.cpp"],
    deps = [
        ":penalty_kick_enemy_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
