package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kick_tactic",
    srcs = ["kick_tactic.cpp"],
    hdrs = [
        "kick_fsm.h",
        "kick_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:kick_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "kick_fsm_test",
    srcs = ["kick_fsm_test.cpp"],
    deps = [
        ":kick_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
