package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "ball_placement_play",
    srcs = ["ball_placement_play.cpp"],
    hdrs = ["ball_placement_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "corner_kick_play",
    srcs = ["corner_kick_play.cpp"],
    hdrs = ["corner_kick_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:defense_shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/logger",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic:shadow_free_kicker_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/logger",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/move:move_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic:kickoff_chip_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/move_goalie_to_goal_line:move_goalie_to_goal_line_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic:penalty_setup_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/penalty_kick:penalty_kick_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:find_open_areas",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/logger",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:calc_best_shot",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic:receiver_tactic",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass_generator",
        "//software/geom/algorithms",
        "//software/logger",
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
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
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
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic",
        "@boost//:coroutine2",
    ],
)

cc_library(
    name = "all_plays",
    deps = [
        ":ball_placement_play",
        ":corner_kick_play",
        ":defense_play",
        ":dribbling_parcour_play",
        ":enemy_free_kick_play",
        ":example_play",
        ":free_kick_play",
        ":halt_play",
        ":kickoff_enemy_play",
        ":kickoff_friendly_play",
        ":pass_endurance_play",
        ":penalty_kick_enemy_play",
        ":penalty_kick_play",
        ":scoring_from_contested_possession_play",
        ":scoring_with_static_defenders_play",
        ":shoot_or_chip_play",
        ":shoot_or_pass_play",
        ":stop_play",
    ],
)

cc_test(
    name = "example_play_test",
    srcs = ["example_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:example_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "kickoff_friendly_play_test",
    srcs = ["kickoff_friendly_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:kickoff_friendly_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "kickoff_enemy_play_test",
    srcs = ["kickoff_enemy_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:kickoff_enemy_play",
        "//software/geom/algorithms",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "corner_kick_play_test",
    srcs = ["corner_kick_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:corner_kick_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "defense_play_test",
    srcs = ["defense_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:defense_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "enemy_free_kick_play_test",
    srcs = ["enemy_free_kick_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:enemy_free_kick_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "stop_play_test",
    srcs = ["stop_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:stop_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "halt_play_test",
    srcs = ["halt_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:halt_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "free_kick_play_test",
    srcs = ["free_kick_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:free_kick_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "shoot_or_pass_play_test",
    srcs = ["shoot_or_pass_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:shoot_or_pass_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "shoot_or_chip_play_test",
    srcs = ["shoot_or_chip_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:shoot_or_chip_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "penalty_kick_play_test",
    srcs = ["penalty_kick_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:penalty_kick_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "penalty_kick_enemy_play_test",
    srcs = ["penalty_kick_enemy_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:penalty_kick_enemy_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_test(
    name = "ball_placement_play_test",
    srcs = ["ball_placement_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:ball_placement_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "scoring_with_static_defenders_play",
    srcs = ["scoring_with_static_defenders_play.cpp"],
    hdrs = ["scoring_with_static_defenders_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "scoring_with_static_defenders_play_test",
    srcs = ["scoring_with_static_defenders_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:scoring_with_static_defenders_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "scoring_from_contested_possession_play",
    srcs = ["scoring_from_contested_possession_play.cpp"],
    hdrs = ["scoring_from_contested_possession_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "scoring_from_contested_possession_play_test",
    srcs = ["scoring_from_contested_possession_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:scoring_from_contested_possession_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "dribbling_parcour_play",
    srcs = ["dribbling_parcour_play.cpp"],
    hdrs = ["dribbling_parcour_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "dribbling_parcour_play_test",
    srcs = ["dribbling_parcour_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:dribbling_parcour_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "pass_endurance_play",
    srcs = ["pass_endurance_play.cpp"],
    hdrs = ["pass_endurance_play.h"],
    deps = [
        ":play",
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/design_patterns:generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "pass_endurance_play_test",
    srcs = ["pass_endurance_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:pass_endurance_play",
        "//software/simulated_tests:simulated_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
