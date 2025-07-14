package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")
# We force linking for all plays so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "penalty_kick_play",
    srcs = [
        "penalty_kick_play.cpp",
        "penalty_kick_play_fsm.cpp",
    ],
    hdrs = [
        "penalty_kick_play.h",
        "penalty_kick_play_fsm.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/penalty_kick:penalty_kick_tactic",
        "//software/logger",
        "//software/util/generic_factory",
        "@sml",
    ],
    alwayslink = True,
)

cc_test(
    name = "penalty_kick_play_fsm_test",
    srcs = ["penalty_kick_play_fsm_test.cpp"],
    deps = [
        ":penalty_kick_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)


py_test(
    name = "penalty_kick_play_test",
    srcs = ["penalty_kick_play_test.py"],
    tags = ["exclusive"],
    deps = [
        "//software:conftest",
        "//software/simulated_tests:validation",
        requirement("pytest"),
    ],
)
