load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pass_defender_tactic",
    srcs = [
        "pass_defender_fsm.cpp",
        "pass_defender_tactic.cpp",
    ],
    hdrs = [
        "pass_defender_fsm.h",
        "pass_defender_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:move_primitive",
        "//software/ai/hl/stp/tactic/defender:defender_fsm_base",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "pass_defender_fsm_test",
    srcs = ["pass_defender_fsm_test.cpp"],
    deps = [
        ":pass_defender_tactic",
        "//software/test_util",
        "@googletest//:gtest_main",
    ],
)

py_test(
    name = "pass_defender_tactic_test",
    srcs = [
        "pass_defender_tactic_test.py",
    ],
    env = {
        "LD_LIBRARY_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/lib",
        "QTWEBENGINEPROCESS_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/libexec/QtWebEngineProcess",
    },
    package_collisions = "warning",
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    deps = [
        "//software:conftest",
        "//software/simulated_tests:speed_threshold_helpers",
        "//software/simulated_tests:validation",
        requirement("pytest"),
        requirement("pyqt6"),
        requirement("numpy"),
        requirement("pyqtdarktheme-fork"),
        requirement("pyqtgraph"),
    ],
)
