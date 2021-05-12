package(default_visibility = ["//visibility:public"])

cc_library(
    name = "move_to_goal_line_tactic",
    srcs = ["move_to_goal_line_tactic.cpp"],
    hdrs = [
        "move_to_goal_line_fsm.h",
        "move_to_goal_line_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/geom:line",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "move_to_goal_line_fms_test",
    srcs = ["move_to_goal_line_fsm_test.cpp"],
    deps = [
        ":move_to_goal_line_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
