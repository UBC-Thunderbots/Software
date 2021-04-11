package(default_visibility = ["//visibility:public"])

cc_library(
    name = "crease_defender_tactic",
    srcs = ["crease_defender_tactic.cpp"],
    hdrs = [
        "crease_defender_fsm.h",
        "crease_defender_tactic.h",
    ],
    deps = [
        "//shared:constants",
        "//shared/parameter:cpp_configs",
        "//software/ai/evaluation:enemy_threat",
        "//software/ai/hl/stp/action:move_action",
        "//software/ai/hl/stp/action:stop_action",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/geom/algorithms",
        "//software/logger",
    ],
)

cc_test(
    name = "crease_defender_tactic_test",
    srcs = ["crease_defender_tactic_test.cpp"],
    deps = [
        ":crease_defender_tactic",
        "//shared:constants",
        "//software/ai/intent:move_intent",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
