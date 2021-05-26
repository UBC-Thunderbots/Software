package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_tactic",
    srcs = ["stop_tactic.cpp"],
    hdrs = [
        "stop_fsm.h",
        "stop_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/intent:move_intent",
        "//software/ai/intent:stop_intent",
        "//software/logger",
    ],
)

cc_test(
    name = "stop_fsm_test",
    srcs = ["stop_fsm_test.cpp"],
    deps = [
        ":stop_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_test(
    name = "stop_tactic_test",
    srcs = ["stop_tactic_test.cpp"],
    deps = [
        ":stop_tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
