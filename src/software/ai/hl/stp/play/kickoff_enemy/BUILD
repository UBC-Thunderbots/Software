load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kickoff_enemy_play",
    srcs = [
        "kickoff_enemy_play.cpp",
        "kickoff_enemy_play_fsm.cpp",
    ],
    hdrs = [
        "kickoff_enemy_play.h",
        "kickoff_enemy_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)
