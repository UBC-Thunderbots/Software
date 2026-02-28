load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "enemy_free_kick_play",
    srcs = [
        "enemy_free_kick_play.cpp",
        "enemy_free_kick_play_fsm.cpp",
    ],
    hdrs = [
        "enemy_free_kick_play.h",
        "enemy_free_kick_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:defender_assignment",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/play/defense:defense_play_base",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/halt:halt_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/pass_defender:pass_defender_tactic",
        "//software/logger",
        "//software/util/generic_factory",
        "//software/world:game_state",
        "@sml",
    ],
    alwayslink = True,
)

py_test(
    name = "enemy_free_kick_play_test",
    srcs = [
        "enemy_free_kick_play_test.py",
    ],
    env = {
        "LD_LIBRARY_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/lib",
        "QTWEBENGINEPROCESS_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/libexec/QtWebEngineProcess",
    },
    package_collisions = "warning",
    tags = [
        "exclusive",
    ],
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
