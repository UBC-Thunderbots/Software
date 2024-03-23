package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "enemy_free_kick_play",
    srcs = [
        "enemy_free_kick_play_fsm.cpp",
        "enemy_free_kick_play.cpp",
        ],
    hdrs = [
        "enemy_free_kick_play_fsm.h",
        "enemy_free_kick_play.h",
        ],
    deps = [
        "//software/ai/hl/stp/play:play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
        "//software/logger",
        "//software/util/generic_factory",
        "//software/world:game_state",
        "//software/ai/hl/stp/play/defense:defense_play",
         "@sml"
    ],
    alwayslink = True,
)


py_test(
    name = "enemy_free_kick_play_test",
    srcs = [
        "enemy_free_kick_play_test.py",
    ],

    tags = [
        "exclusive",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)