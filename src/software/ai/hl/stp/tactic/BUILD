package(default_visibility = ["//visibility:public"])

cc_library(
    name = "block_shot_path_tactic",
    srcs = ["block_shot_path_tactic.cpp"],
    hdrs = ["block_shot_path_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/geom",
        "@g3log",
    ],
)

cc_test(
    name = "block_shot_path_tactic_test",
    srcs = ["block_shot_path_tactic_test.cpp"],
    deps = [
        ":block_shot_path_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "cherry_pick_tactic",
    srcs = ["cherry_pick_tactic.cpp"],
    hdrs = ["cherry_pick_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass_generator",
        "//software/geom",
        "//software/world",
        "@g3log",
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
        "//software/geom",
        "@g3log",
    ],
)

cc_library(
    name = "crease_defender_tactic",
    srcs = ["crease_defender_tactic.cpp"],
    hdrs = ["crease_defender_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/geom",
        "@g3log",
    ],
)

cc_library(
    name = "goalie_tactic",
    srcs = ["goalie_tactic.cpp"],
    hdrs = ["goalie_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/geom",
        "@g3log",
    ],
)

cc_library(
    name = "move_tactic",
    srcs = ["move_tactic.cpp"],
    hdrs = ["move_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/geom",
        "@g3log",
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

cc_library(
    name = "passer_tactic",
    srcs = ["passer_tactic.cpp"],
    hdrs = ["passer_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/passing:pass",
        "//software/geom",
        "@g3log",
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
    name = "patrol_tactic",
    srcs = ["patrol_tactic.cpp"],
    hdrs = ["patrol_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom",
        "@g3log",
    ],
)

cc_library(
    name = "penalty_kick_tactic",
    srcs = ["penalty_kick_tactic.cpp"],
    hdrs = ["penalty_kick_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:dribble_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/evaluation:calc_best_shot",
        "//software/geom",
        "@g3log",
    ],
)

cc_library(
    name = "receiver_tactic",
    srcs = ["receiver_tactic.cpp"],
    hdrs = ["receiver_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/evaluation:calc_best_shot",
        "//software/ai/passing:pass",
        "//software/geom",
        "@g3log",
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
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/geom",
        "@g3log",
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
    name = "shadow_freekicker_tactic",
    srcs = ["shadow_freekicker_tactic.cpp"],
    hdrs = ["shadow_freekicker_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/evaluation:possession",
        "//software/geom",
        "@g3log",
    ],
)

cc_library(
    name = "shoot_goal_tactic",
    srcs = ["shoot_goal_tactic.cpp"],
    hdrs = ["shoot_goal_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/evaluation:calc_best_shot",
        "//software/ai/hl/stp/evaluation:intercept",
        "//software/geom",
        "@g3log",
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
    hdrs = ["stop_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom",
        "@g3log",
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
    name = "tactic",
    srcs = ["tactic.cpp"],
    hdrs = ["tactic.h"],
    deps = [
        "//software/ai/hl/stp/action",
        "//software/world",
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
