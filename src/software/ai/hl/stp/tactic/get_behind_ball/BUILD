package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_behind_ball_tactic",
    srcs = ["get_behind_ball_tactic.cpp"],
    hdrs = [
        "get_behind_ball_fsm.h",
        "get_behind_ball_tactic.h",
    ],
    deps = [
        "//software/ai/hl/stp/tactic",
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/intent:move_intent",
        "//software/geom:triangle",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "get_behind_ball_fsm_test",
    srcs = ["get_behind_ball_fsm_test.cpp"],
    deps = [
        ":get_behind_ball_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
