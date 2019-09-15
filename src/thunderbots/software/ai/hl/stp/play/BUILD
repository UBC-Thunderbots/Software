package(default_visibility = ["//visibility:public"])

cc_library(
    name = "corner_kick_play",
    srcs = ["corner_kick_play.cpp"],
    hdrs = ["corner_kick_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/evaluation:ball",
        "//software/ai/hl/stp/evaluation:possession",
        "//software/ai/hl/stp/tactic:cherry_pick_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:passer_tactic",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/passing:pass_generator",
        "@g3log//:all",
    ],
)

cc_library(
    name = "defense_play",
    srcs = ["defense_play.cpp"],
    hdrs = ["defense_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/ai/hl/stp/evaluation:possession",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/ai/world:game_state",
        "//software/util/parameter:dynamic_parameters",
        "@g3log//:all",
    ],
)

cc_library(
    name = "enemy_freekick_play",
    srcs = ["enemy_freekick_play.cpp"],
    hdrs = ["enemy_freekick_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic:block_shot_path_tactic",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:shadow_freekicker_tactic",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/ai/world:game_state",
        "//software/util/parameter:dynamic_parameters",
        "@g3log//:all",
    ],
)

cc_library(
    name = "example_play",
    srcs = ["example_play.cpp"],
    hdrs = ["example_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:move_tactic",
        "@g3log//:all",
    ],
)

cc_test(
    name = "example_play_test",
    srcs = [
        "example_play_test.cpp",
    ],
    deps = [
        ":example_play",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/test/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "free_kick_play",
    srcs = ["free_kick_play.cpp"],
    hdrs = ["free_kick_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/evaluation:calc_best_shot",
        "//software/ai/hl/stp/evaluation:possession",
        "//software/ai/hl/stp/tactic:cherry_pick_tactic",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:passer_tactic",
        "//software/ai/hl/stp/tactic:patrol_tactic",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/hl/stp/tactic:shoot_goal_tactic",
        "//software/ai/passing:pass_generator",
        "//software/util/logger",
        "@g3log//:all",
    ],
)

cc_library(
    name = "halt_play",
    srcs = ["halt_play.cpp"],
    hdrs = ["halt_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "@g3log//:all",
    ],
)

cc_test(
    name = "halt_play_test",
    srcs = ["halt_play_test.cpp"],
    deps = [
        ":halt_play",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/test/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "kickoff_enemy_play",
    srcs = ["kickoff_enemy_play.cpp"],
    hdrs = ["kickoff_enemy_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/ai/hl/stp/evaluation:possession",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/util/logger",
        "@g3log//:all",
    ],
)

cc_library(
    name = "kickoff_friendly_play",
    srcs = ["kickoff_friendly_play.cpp"],
    hdrs = ["kickoff_friendly_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:chip_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "@g3log//:all",
    ],
)

cc_library(
    name = "penalty_kick_enemy_play",
    srcs = ["penalty_kick_enemy_play.cpp"],
    hdrs = ["penalty_kick_enemy_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "@g3log//:all",
    ],
)

cc_library(
    name = "penalty_kick_play",
    srcs = ["penalty_kick_play.cpp"],
    hdrs = ["penalty_kick_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:penalty_kick_tactic",
        "@g3log//:all",
    ],
)

cc_library(
    name = "shoot_or_chip_play",
    srcs = ["shoot_or_chip_play.cpp"],
    hdrs = ["shoot_or_chip_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/evaluation:enemy_threat",
        "//software/ai/hl/stp/evaluation:find_open_areas",
        "//software/ai/hl/stp/evaluation:indirect_chip",
        "//software/ai/hl/stp/evaluation:possession",
        "//software/ai/hl/stp/tactic:crease_defender_tactic",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "//software/ai/hl/stp/tactic:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic:shoot_goal_tactic",
        "//software/ai/hl/stp/tactic:stop_tactic",
        "//software/ai/world:game_state",
        "//software/util/parameter:dynamic_parameters",
        "@g3log//:all",
    ],
)

cc_library(
    name = "stop_play",
    srcs = ["stop_play.cpp"],
    hdrs = ["stop_play.h"],
    deps = [
        ":play",
        ":play_factory",
        "//shared:constants",
        "//software/ai/hl/stp/tactic:goalie_tactic",
        "//software/ai/hl/stp/tactic:move_tactic",
        "@g3log//:all",
    ],
)

cc_library(
    name = "play",
    srcs = ["play.cpp"],
    hdrs = ["play.h"],
    deps = ["//software/ai/hl/stp/tactic"],
)

cc_library(
    name = "play_factory",
    srcs = ["play_factory.cpp"],
    hdrs = ["play_factory.h"],
    deps = [":play"],
)

cc_test(
    name = "play_factory_test",
    srcs = ["play_factory_test.cpp"],
    deps = [
        ":play_factory",
        "//software/ai/hl/stp/play/test_plays:halt_test_play",
        "//software/ai/hl/stp/play/test_plays:move_test_play",
        "@gtest//:gtest_main",
    ],
)
