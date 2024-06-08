package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "enemy_ball_placement_play",
    srcs = [
        "enemy_ball_placement_play.cpp",
        "enemy_ball_placement_play_fsm.cpp",
    ],
    hdrs = [
        "enemy_ball_placement_play.h",
        "enemy_ball_placement_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/util/generic_factory",
        "//software/world:game_state",
    ],
    alwayslink = True,
)

py_test(
    name = "enemy_ball_placement_play_test",
    srcs = [
        "enemy_ball_placement_play_test.py",
    ],
    tags = ["exclusive"],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
