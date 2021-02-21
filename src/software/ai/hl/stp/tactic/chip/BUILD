package(default_visibility = ["//visibility:public"])

cc_library(
    name = "chip_tactic",
    srcs = ["chip_tactic.cpp"],
    hdrs = [
        "chip_fsm.h",
        "chip_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:chip_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/get_behind_ball:get_behind_ball_tactic",
        "//software/logger",
    ],
)

cc_test(
    name = "chip_fsm_test",
    srcs = ["chip_fsm_test.cpp"],
    deps = [
        ":chip_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "chip_tactic_test",
    srcs = ["chip_tactic_test.cpp"],
    deps = [
        ":chip_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
