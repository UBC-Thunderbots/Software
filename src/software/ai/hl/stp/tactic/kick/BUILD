load("@simulated_tests_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kick_tactic",
    srcs = [
        "kick_fsm.cpp",
        "kick_tactic.cpp",
    ],
    hdrs = [
        "kick_fsm.h",
        "kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:move_primitive",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "kick_fsm_test",
    srcs = ["kick_fsm_test.cpp"],
    deps = [
        ":kick_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
