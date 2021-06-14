package(default_visibility = ["//visibility:public"])

cc_test(
    name = "penalty_setup_tactic_test",
    srcs = ["penalty_setup_tactic_test.cpp"],
    deps = [
        ":penalty_setup_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "kickoff_chip_tactic",
    srcs = ["kickoff_chip_tactic.cpp"],
    hdrs = ["kickoff_chip_tactic.h"],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/logger",
    ],
)

cc_library(
    name = "defense_shadow_enemy_tactic",
    srcs = ["defense_shadow_enemy_tactic.cpp"],
    hdrs = ["defense_shadow_enemy_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/logger",
    ],
)

cc_test(
    name = "defense_shadow_enemy_tactic_test",
    srcs = ["defense_shadow_enemy_tactic_test.cpp"],
    deps = [
        ":defense_shadow_enemy_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "penalty_setup_tactic",
    srcs = ["penalty_setup_tactic.cpp"],
    hdrs = ["penalty_setup_tactic.h"],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
    ],
)

cc_library(
    name = "receiver_tactic",
    srcs = ["receiver_tactic.cpp"],
    hdrs = ["receiver_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_library(
    name = "shadow_free_kicker_tactic",
    srcs = ["shadow_free_kicker_tactic.cpp"],
    hdrs = ["shadow_free_kicker_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/action:move_action",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "shadow_free_kicker_tactic_test",
    srcs = ["shadow_free_kicker_tactic_test.cpp"],
    deps = [
        ":shadow_free_kicker_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "all_tactics",
    hdrs = ["all_tactics.h"],
    deps = [
        ":defense_shadow_enemy_tactic",
        ":kickoff_chip_tactic",
        ":penalty_setup_tactic",
        ":receiver_tactic",
        ":shadow_free_kicker_tactic",
        ":tactic",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/move_goalie_to_goal_line:move_goalie_to_goal_line_tactic",
        "//software/ai/hl/stp/tactic/penalty_kick:penalty_kick_tactic",
        "//software/ai/hl/stp/tactic/pivot_kick:pivot_kick_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
    ],
)

cc_library(
    name = "tactic",
    srcs = ["tactic.cpp"],
    hdrs = [
        "tactic.h",
        "tactic_fsm.h",
        "tactic_visitor.h",
    ],
    deps = [
        ":transition_conditions",
        "//software/ai/hl/stp/action",
        "//software/ai/intent",
        "//software/ai/intent:stop_intent",
        "//software/util/typename",
        "//software/world",
        "@sml",
    ],
)

cc_test(
    name = "tactic_test",
    srcs = ["tactic_test.cpp"],
    deps = [
        ":tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/test_tactics:move_test_tactic",
        "//software/test_util",
    ],
)

cc_library(
    name = "transition_conditions",
    srcs = ["transition_conditions.cpp"],
    hdrs = ["transition_conditions.h"],
    deps = [
        "//software/world",
    ],
)

cc_test(
    name = "transition_conditions_test",
    srcs = ["transition_conditions_test.cpp"],
    deps = [
        ":tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/world",
    ],
)
