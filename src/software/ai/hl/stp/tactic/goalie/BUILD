package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "goalie_tactic",
    srcs = [
        "goalie_fsm.cpp",
        "goalie_tactic.cpp",
    ],
    hdrs = [
        "goalie_fsm.h",
        "goalie_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:find_open_areas",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/dribble:dribble_tactic",
        "//software/ai/hl/stp/tactic/pivot_kick:pivot_kick_tactic",
        "//software/geom:line",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/math:math_functions",
    ],
)

cc_test(
    name = "goalie_fsm_test",
    srcs = ["goalie_fsm_test.cpp"],
    deps = [
        ":goalie_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "goalie_tactic_chip_test",
    srcs = ["goalie_tactic_chip_test.cpp"],
    deps = [
        ":goalie_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

py_test(
    name = "goalie_tactic_test",
    srcs = [
        "goalie_tactic_test.py",
    ],
    deps = [
        "//software/simulated_tests:simulated_test_fixture",
        "//software/simulated_tests:speed_threshold_helpers",
        "//software/simulated_tests:test_constants",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
