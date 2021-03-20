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
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom/algorithms",
        "//software/logger",
    ]
)