package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kick_tactic",
    srcs = ["kick_tactic.cpp"],
    hdrs = [
        "kick_fsm.h",
        "kick_tactic.h",
    ],
    deps = [
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/ai/hl/stp/tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:kick_action",
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
