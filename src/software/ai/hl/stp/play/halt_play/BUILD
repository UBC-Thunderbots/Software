load("@simulated_tests_deps//:requirements.bzl", "requirement")
load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "halt_play",
    srcs = [
        "halt_play.cpp",
        "halt_play_fsm.cpp",
    ],
    hdrs = [
        "halt_play.h",
        "halt_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/halt:halt_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "halt_play_fsm_test",
    srcs = ["halt_play_fsm_test.cpp"],
    deps = [
        ":halt_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "halt_play_test",
    srcs = [
        "halt_play_test.py",
    ],
    # TODO (#2619) Remove tag to run in parallel
    tags = [
        "exclusive",
    ],
    env = {
        "LD_LIBRARY_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/lib",
        "QTWEBENGINEPROCESS_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/libexec/QtWebEngineProcess",
    },
    package_collisions = "warning",
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
