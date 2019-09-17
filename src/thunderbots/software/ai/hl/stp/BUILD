package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stp",
    srcs = ["stp.cpp"],
    hdrs = ["stp.h"],
    # We include all the plays here so that that the static
    # variables in the .cpp files that auto-register plays with the
    # PlayFactory. This addresses the issue explained here:
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    deps = [
        "//software/ai/hl",
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/play:play_factory",
        "//software/ai/hl/stp/play:stop_play",
        "//software/ai/hl/stp/play:halt_play",
        "//software/ai/hl/stp/tactic",
        "//software/ai/intent:stop_intent",
        "//software/util/parameter:dynamic_parameters",
    ] + glob(["play/*.cpp"]),
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
        "//software/ai/world",
        "//software/test_util",
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
