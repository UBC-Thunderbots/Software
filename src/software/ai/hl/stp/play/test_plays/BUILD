package(default_visibility = ["//visibility:public"])

cc_library(
    name = "example_play",
    srcs = ["example_play.cpp"],
    hdrs = ["example_play.h"],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/skill/move:move_skill",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactic",
        "//software/logger",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "example_play_test",
    srcs = ["example_play_test.cpp"],
    deps = [
        ":example_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/simulated_tests:simulated_er_force_sim_play_test_fixture",
        "//software/simulated_tests/validation:validation_function",
        "//software/test_util",
        "//software/time:duration",
        "//software/world",
    ],
)

cc_library(
    name = "halt_test_play",
    srcs = ["halt_test_play.cpp"],
    hdrs = ["halt_test_play.h"],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/skill/stop:stop_skill",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactic",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "move_test_play",
    srcs = ["move_test_play.cpp"],
    hdrs = ["move_test_play.h"],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/skill/move:move_skill",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactic",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)
