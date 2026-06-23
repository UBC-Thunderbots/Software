load("@gameplay_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

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
    # Full-system ball-placement sim test; intermittently flaky under parallel load (like
    # the ball_placement / kickoff_enemy siblings, which are also flaky). Retry rather than
    # gate CI on a single flake; it passes reliably on its own.
    flaky = True,
    deps = [
        "//software:conftest",
        "//software/gameplay_tests/validation:validations",
        requirement("pytest"),
    ],
)
