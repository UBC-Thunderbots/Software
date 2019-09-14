package(default_visibility = ["//visibility:public"])

cc_library(
    name = "block_shot_path_tactic",
    srcs = ["block_shot_path_tactic.cpp"],
    hdrs = ["block_shot_path_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/ai/world",
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
    ],
)

cc_library(
    name = "free_kick_tactic",
    srcs = ["free_kick_tactic.cpp"],
    hdrs = ["free_kick_tactic.h"],
    deps = [
        ":tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/evaluation:calc_best_shot",
        "//software/ai/hl/stp/evaluation:deflect_off_enemy_target",
        "//software/ai/hl/stp/evaluation:indirect_chip",
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
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
        "//software/geom:all",
        "@g3log//:all",
    ],
)

cc_library(
    name = "tactic",
    srcs = ["tactic.cpp"],
    hdrs = ["tactic.h"],
    deps = [
        "//software/ai/hl/stp/action",
        "//software/ai/world",
    ],
)
