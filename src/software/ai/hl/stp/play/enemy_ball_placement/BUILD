load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
load("@simulated_tests_deps//:requirements.bzl", "requirement")

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
    env = {
        "LD_LIBRARY_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/lib",
        "QTWEBENGINEPROCESS_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/libexec/QtWebEngineProcess",
    },
    package_collisions = "warning",
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/simulated_tests:validation",
        requirement("pytest"),
        requirement("pyqt6"),
        requirement("numpy"),
        requirement("pyqtdarktheme-fork"),
        requirement("pyqtgraph"),
    ],
)
