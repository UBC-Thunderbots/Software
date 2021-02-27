package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stp",
    srcs = [
        "stp.cpp",
    ],
    hdrs = ["stp.h"],
    deps = [
        "//software/ai/hl",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:all_tactics",
        "//software/ai/intent:stop_intent",
        "//software/ai/motion_constraint:motion_constraint_set_builder",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "//software/util/typename",
        "@munkres_cpp",
    ],
)

cc_library(
    name = "play_info",
    srcs = ["play_info.cpp"],
    hdrs = ["play_info.h"],
    deps = [],
)

cc_test(
    name = "play_info_test",
    srcs = ["play_info_test.cpp"],
    deps = [
        ":play_info",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "stp_referee_command_play_selection_test",
    srcs = ["stp_referee_command_play_selection_test.cpp"],
    deps = [
        ":stp",
        "//software/ai/hl/stp/play:all_plays",
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
        "//software/ai/hl/stp/play:halt_play",
        "//software/ai/hl/stp/play/test_plays:halt_test_play",
        "//software/ai/hl/stp/tactic/test_tactics:goalie_test_tactic",
        "//software/ai/hl/stp/tactic/test_tactics:move_test_tactic",
        "//software/ai/hl/stp/tactic/test_tactics:stop_test_tactic",
        "//software/parameter:dynamic_parameters",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "stp_test",
    srcs = ["stp_test.cpp"],
    deps = [
        ":stp",
        "//software/ai/hl/stp/play:halt_play",
        "//software/ai/hl/stp/play/test_plays:halt_test_play",
        "//software/ai/hl/stp/play/test_plays:move_test_play",
        "//software/test_util",
        "//software/util/design_patterns:generic_factory",
        "@gtest//:gtest_main",
    ],
)
