package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "corner_kick_play",
    srcs = ["corner_kick_play.cpp"],
    hdrs = ["corner_kick_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:cherry_pick_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:passer_tactic",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/passing:pass_generator",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
        "//software/world:ball",
    ],
    alwayslink = True,
)

cc_library(
    name = "defense_play",
    srcs = ["defense_play.cpp"],
    hdrs = ["defense_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:defense_shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:shoot_goal_tactic",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "//software/world:game_state",
        "//software/world:team",
    ],
    alwayslink = True,
)

cc_library(
    name = "enemy_free_kick_play",
    srcs = ["enemy_free_kick_play.cpp"],
    hdrs = ["enemy_free_kick_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:shadow_free_kicker_tactic",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "//software/world:game_state",
    ],
    alwayslink = True,
)

cc_library(
    name = "example_play",
    srcs = ["example_play.cpp"],
    hdrs = ["example_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "halt_play",
    srcs = ["halt_play.cpp"],
    hdrs = ["halt_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "free_kick_play",
    srcs = ["free_kick_play.cpp"],
    hdrs = ["free_kick_play.h"],
    deps = [
        ":corner_kick_play",
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:cherry_pick_tactic",
        "//software/ai/hl/stp/tactic:chip_tactic",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:passer_tactic",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/hl/stp/tactic:shoot_goal_tactic",
        "//software/ai/passing:pass_generator",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
        "//software/world:ball",
    ],
    alwayslink = True,
)

cc_library(
    name = "kickoff_enemy_play",
    srcs = ["kickoff_enemy_play.cpp"],
    hdrs = ["kickoff_enemy_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "kickoff_friendly_play",
    srcs = ["kickoff_friendly_play.cpp"],
    hdrs = ["kickoff_friendly_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:kickoff_chip_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "penalty_kick_enemy_play",
    srcs = ["penalty_kick_enemy_play.cpp"],
    hdrs = ["penalty_kick_enemy_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "penalty_kick_play",
    srcs = ["penalty_kick_play.cpp"],
    hdrs = ["penalty_kick_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:penalty_kick_tactic",
        "//software/ai/hl/stp/tactic:penalty_setup_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "shoot_or_chip_play",
    srcs = ["shoot_or_chip_play.cpp"],
    hdrs = ["shoot_or_chip_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:find_open_areas",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:shoot_goal_tactic",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "//software/world:game_state",
    ],
    alwayslink = True,
)

cc_library(
    name = "shoot_or_pass_play",
    srcs = ["shoot_or_pass_play.cpp"],
    hdrs = ["shoot_or_pass_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:cherry_pick_tactic",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:passer_tactic",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:shoot_goal_tactic",
        "//software/ai/passing:pass_generator",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "stop_play",
    srcs = ["stop_play.cpp"],
    hdrs = ["stop_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "play",
    srcs = ["play.cpp"],
    hdrs = ["play.h"],
    deps = [
        "//software/ai/hl/stp/tactic",
        "@boost//:coroutine2",
    ],
)

cc_library(
    name = "all_plays",
    deps = [
        ":corner_kick_play",
        ":defense_play",
        ":enemy_free_kick_play",
        ":example_play",
        ":free_kick_play",
        ":halt_play",
        ":kickoff_enemy_play",
        ":kickoff_friendly_play",
        ":penalty_kick_enemy_play",
        ":penalty_kick_play",
        ":shoot_or_chip_play",
        ":shoot_or_pass_play",
        ":stop_play",
    ],
)
