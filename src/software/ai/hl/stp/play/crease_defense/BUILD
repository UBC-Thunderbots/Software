load("@simulated_tests_deps//:requirements.bzl", "requirement")
load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
package(default_visibility = ["//visibility:public"])

# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "crease_defense_play",
    srcs = [
        "crease_defense_play.cpp",
        "crease_defense_play_fsm.cpp",
    ],
    hdrs = [
        "crease_defense_play.h",
        "crease_defense_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
    ],
    alwayslink = True,
)

py_test(
    name = "crease_defense_play_test",
    srcs = [
        "crease_defense_play_test.py",
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
        "//software/simulated_tests:validation",
        requirement("pytest"),
        requirement("pyqt6"),
        requirement("numpy"),
        requirement("pyqtdarktheme-fork"),
        requirement("pyqtgraph"),
    ],
)

cc_test(
    name = "crease_defense_play_cpp_test",
    srcs = ["crease_defense_play_test.cpp"],
    deps = [
        ":crease_defense_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
    ],
)

cc_test(
    name = "crease_defense_play_fsm_test",
    srcs = ["crease_defense_play_fsm_test.cpp"],
    deps = [
        ":crease_defense_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
