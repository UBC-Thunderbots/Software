package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

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
        "//software/ai/hl/stp/play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/tactic/chip:chip_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/evaluation:find_open_areas",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "kickoff_friendly_play_cpp_test",
    srcs = ["kickoff_friendly_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play/kickoff_friendly:kickoff_friendly_play",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)
