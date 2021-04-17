package(default_visibility = ["//visibility:public"])

cc_library(
    name = "penalty_kick_tactic2",
    srcs = ["penalty_kick_tactic.cpp"],
    hdrs = [
        "penalty_kick_tactic2.h",
        "penalty_kick_tactic_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom/algorithms",
        "//software/logger",
    ]
)

cc_test(
    name = "penalty_kick_tactic_fsm_test",
    srcs = ["penalty_kick_tactic_fsm_test.cpp"],
    deps = [
        "@gtest//:gtest_main",
        "//software/test_util",
        ":penalty_kick_tactic2",
    ]
)