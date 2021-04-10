package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cherry_pick_tactic",
    srcs = ["cherry_pick_tactic.cpp"],
    hdrs = ["cherry_pick_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/world",
    ],
)

cc_test(
    name = "penalty_setup_tactic_test",
    srcs = ["penalty_setup_tactic_test.cpp"],
    deps = [
        ":penalty_setup_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
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
    name = "crease_defender_tactic",
    srcs = ["crease_defender_tactic.cpp"],
    hdrs = ["crease_defender_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "crease_defender_tactic_test",
    srcs = ["crease_defender_tactic_test.cpp"],
    deps = [
        ":crease_defender_tactic",
        "//shared:constants",
        "//software/ai/intent:move_intent",
        "//software/test_util",
        "@gtest//:gtest_main",
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
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "passer_tactic",
    srcs = ["passer_tactic.cpp"],
    hdrs = ["passer_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass",
        "//software/logger",
        "//software/world:ball",
    ],
)

cc_library(
    name = "penalty_kick_tactic",
    srcs = ["penalty_kick_tactic.cpp"],
    hdrs = ["penalty_kick_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "penalty_kick_tactic_test",
    srcs = ["penalty_kick_tactic_test.cpp"],
    deps = [
        ":penalty_kick_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
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
    name = "shadow_enemy_tactic",
    srcs = ["shadow_enemy_tactic.cpp"],
    hdrs = ["shadow_enemy_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/logger",
    ],
)

cc_test(
    name = "shadow_enemy_tactic_test",
    srcs = ["shadow_enemy_tactic_test.cpp"],
    deps = [
        ":shadow_enemy_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
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
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "shoot_goal_tactic",
    srcs = ["shoot_goal_tactic.cpp"],
    hdrs = ["shoot_goal_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/logger",
    ],
)

cc_test(
    name = "shoot_goal_tactic_test",
    srcs = ["shoot_goal_tactic_test.cpp"],
    deps = [
        ":shoot_goal_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "all_tactics",
    hdrs = ["all_tactics.h"],
    deps = [
        ":cherry_pick_tactic",
        ":crease_defender_tactic",
        ":defense_shadow_enemy_tactic",
        ":goalie_tactic",
        ":kickoff_chip_tactic",
        ":passer_tactic",
        ":penalty_kick_tactic",
        ":penalty_setup_tactic",
        ":receiver_tactic",
        ":shadow_enemy_tactic",
        ":shadow_free_kicker_tactic",
        ":shoot_goal_tactic",
        ":tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/ai/hl/stp/tactic/kick:kick_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
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
        "//software/ai/hl/stp/tactic/test_tactics:move_test_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
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

cc_library(
    name = "transition_conditions_test",
    srcs = ["transition_conditions_test.cpp"],
    deps = [
        ":tactic",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
