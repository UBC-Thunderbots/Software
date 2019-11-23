package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stp",
    srcs = ["stp.cpp"],
    hdrs = ["stp.h"],
    deps = [
        "//software/ai/hl",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/play:play_factory",
        "//software/ai/hl/stp/tactic",
        "//software/ai/intent:stop_intent",
        "//software/util/parameter:dynamic_parameters",
        "//software/ai/motion_constraint:motion_constraint_manager",
        "@munkres_cpp",
    ],
)

cc_library(
    name = "play_info",
    hdrs = ["play_info.h"],
    deps = [],
)

cc_test(
    name = "stp_refbox_game_state_play_selection_test",
    srcs = ["stp_refbox_game_state_play_selection_test.cpp"],
    deps = [
        ":stp",
        "//software/ai/hl/stp/play:halt_play",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "stp_tactic_assignment_test",
    srcs = ["stp_tactic_assignment_test.cpp"],
    deps = [
        ":stp",
        "//software/ai/hl/stp/play/test_plays:halt_test_play",
        "//software/ai/hl/stp/tactic/test_tactics:move_test_tactic",
        "//software/ai/hl/stp/tactic/test_tactics:stop_test_tactic",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "stp_test",
    srcs = ["stp_test.cpp"],
    deps = [
        ":stp",
        "//software/ai/hl/stp/play:play_factory",
        "//software/ai/hl/stp/play/test_plays:halt_test_play",
        "//software/ai/hl/stp/play/test_plays:move_test_play",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
