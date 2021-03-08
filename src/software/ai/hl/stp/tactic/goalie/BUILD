package(default_visibility = ["//visibility:public"])

cc_library(
    name = "goalie_tactic",
    srcs = ["goalie_tactic.cpp"],
    hdrs = [
        "goalie_fsm.h",
        "goalie_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/geom:line",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
    ],
)

cc_test(
    name = "goalie_fsm_test",
    srcs = ["goalie_fsm_test.cpp"],
    deps = [
        ":goalie_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "goalie_tactic_test",
    srcs = ["goalie/goalie_tactic_test.cpp"],
    deps = [
        ":goalie_tactic",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
