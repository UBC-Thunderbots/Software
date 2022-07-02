package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_test(
    name = "free_kick_play_test",
    srcs = ["free_kick_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play/free_kick:free_kick_play",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "free_kick_play",
    srcs = [
        "free_kick_play.cpp",
        "free_kick_play_fsm.cpp",
    ],
    hdrs = [
        "free_kick_play.h",
        "free_kick_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:find_open_areas",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/play/crease_defense:crease_defense_play",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/receiver:receiver_tactic",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass_generator",
        "//software/logger",
        "//software/util/generic_factory",
        "//software/world:ball",
    ],
    alwayslink = True,
)
