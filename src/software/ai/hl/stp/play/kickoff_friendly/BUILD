load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kickoff_friendly_play",
    srcs = [
        "kickoff_friendly_play.cpp",
        "kickoff_friendly_play_fsm.cpp",
    ],
    hdrs = [
        "kickoff_friendly_play.h",
        "kickoff_friendly_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:find_open_areas",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)
