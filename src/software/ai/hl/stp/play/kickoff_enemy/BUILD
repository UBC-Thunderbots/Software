package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

cc_library(
    name = "kickoff_enemy_play",
    srcs = [
        "kickoff_enemy_play.cpp",
        "kickoff_enemy_play_fsm.cpp"
    ],
    hdrs = [
        "kickoff_enemy_play.h",
        "kickoff_enemy_play_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/play",
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/evaluation:possession",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)


cc_test(
    name = "kickoff_enemy_play_cpp_test",
    srcs = ["kickoff_enemy_play_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:kickoff_enemy_play",
        "//software/geom/algorithms",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/non_terminating_validation_functions",
        "//software/simulated_tests/terminating_validation_functions",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)