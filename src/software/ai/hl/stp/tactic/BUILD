package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cherry_pick_tactic",
    srcs = ["cherry_pick_tactic.cpp"],
    hdrs = ["cherry_pick_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass_generator",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/world",
    ],
)

cc_test(
    name = "cherry_pick_tactic_test",
    srcs = ["cherry_pick_tactic_test.cpp"],
    deps = [
        ":cherry_pick_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "chip_tactic",
    srcs = ["chip_tactic.cpp"],
    hdrs = ["chip_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
        "//software/logger",
    ],
)

cc_test(
    name = "chip_tactic_test",
    srcs = ["chip_tactic_test.cpp"],
    deps = [
        ":chip_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "kickoff_chip_tactic_test",
    srcs = ["kickoff_chip_tactic_test.cpp"],
    deps = [
        ":kickoff_chip_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
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
        ":chip_tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
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
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:autochip_move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
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
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:autochip_move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
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
    name = "goalie_tactic",
    srcs = ["goalie_tactic.cpp"],
    hdrs = ["goalie_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:autochip_move_action",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom:line",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
    ],
)

cc_test(
    name = "goalie_tactic_test",
    srcs = ["goalie_tactic_test.cpp"],
    deps = [
        ":goalie_tactic",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "intercept_ball_tactic",
    srcs = ["intercept_ball_tactic.cpp"],
    hdrs = [
        "intercept_ball_fsm.h",
        "intercept_ball_tactic.h",
    ],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/intent:move_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "intercept_ball_fsm_test",
    srcs = ["intercept_ball_fsm_test.cpp"],
    deps = [
        ":intercept_ball_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_intercept_ball_tactic_test",
    srcs = ["simulated_intercept_ball_tactic_test.cpp"],
    deps = [
        "//software/ai/hl/stp/tactic:intercept_ball_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/validation:validation_function",
        "//software/simulated_tests/validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "move_tactic",
    srcs = ["move_tactic.cpp"],
    hdrs = [
        "move_fsm.h",
        "move_tactic.h",
    ],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/intent:move_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "move_fsm_test",
    srcs = ["move_fsm_test.cpp"],
    deps = [
        ":move_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "move_tactic_test",
    srcs = ["move_tactic_test.cpp"],
    deps = [
        ":move_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "simulated_move_tactic_test",
    srcs = ["simulated_move_tactic_test.cpp"],
    deps = [
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/simulated_tests:simulated_tactic_test_fixture",
        "//software/simulated_tests:simulated_test_main",
        "//software/simulated_tests/validation:validation_function",
        "//software/simulated_tests/validation_functions",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
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

cc_test(
    name = "passer_tactic_test",
    srcs = ["passer_tactic_test.cpp"],
    deps = [
        ":passer_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
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

cc_library(
    name = "penalty_setup_tactic",
    srcs = ["penalty_setup_tactic.cpp"],
    hdrs = ["penalty_setup_tactic.h"],
    deps = [
        ":move_tactic",
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
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
        "//software/ai/hl/stp/action:autokick_move_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "receiver_tactic_test",
    srcs = ["receiver_tactic_test.cpp"],
    deps = [
        ":receiver_tactic",
        "//software/ai/intent:kick_intent",
        "//software/test_util",
        "@gtest//:gtest_main",
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
        "//software/ai/hl/stp/action:autochip_move_action",
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
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/action:move_action",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
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
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:intercept_ball_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
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
    name = "stop_tactic",
    srcs = ["stop_tactic.cpp"],
    hdrs = [
        "stop_fsm.h",
        "stop_tactic.h",
    ],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/intent:move_intent",
        "//software/ai/intent:stop_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "stop_fsm_test",
    srcs = ["stop_fsm_test.cpp"],
    deps = [
        ":stop_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "stop_tactic_test",
    srcs = ["stop_tactic_test.cpp"],
    deps = [
        ":stop_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "all_tactics",
    hdrs = ["all_tactics.h"],
    deps = [
        ":cherry_pick_tactic",
        ":chip_tactic",
        ":crease_defender_tactic",
        ":defense_shadow_enemy_tactic",
        ":goalie_tactic",
        ":intercept_ball_tactic",
        ":kickoff_chip_tactic",
        ":move_tactic",
        ":passer_tactic",
        ":penalty_kick_tactic",
        ":penalty_setup_tactic",
        ":receiver_tactic",
        ":shadow_enemy_tactic",
        ":shadow_free_kicker_tactic",
        ":shoot_goal_tactic",
        ":stop_tactic",
        ":tactic",
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
