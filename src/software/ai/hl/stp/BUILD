package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stp",
    srcs = [
        "play_selection_fsm.cpp",
        "stp.cpp",
    ],
    hdrs = [
        "play_selection_fsm.h",
        "stp.h",
    ],
    deps = [
        "//proto:play_info_msg_cc_proto",
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/play:all_plays",
        "//software/ai/hl/stp/tactic",
        "//software/ai/hl/stp/tactic:all_tactics",
        "//software/ai/intent",
        "//software/ai/intent:stop_intent",
        "//software/ai/motion_constraint:motion_constraint_set_builder",
        "//software/util/generic_factory",
        "//software/util/sml_fsm",
        "//software/util/typename",
        "//software/world",
        "@munkres_cpp",
    ],
)

cc_test(
    name = "stp_referee_command_play_selection_test",
    srcs = ["stp_referee_command_play_selection_test.cpp"],
    deps = [
        ":stp",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:all_plays",
        "//software/test_util",
        "//software/world",
    ],
)

cc_test(
    name = "stp_tactic_assignment_test",
    srcs = ["stp_tactic_assignment_test.cpp"],
    deps = [
        ":stp",
        "//shared/parameter:cpp_configs",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:halt_play",
        "//software/test_util",
    ],
)

cc_test(
    name = "stp_test",
    srcs = ["stp_test.cpp"],
    deps = [
        ":stp",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play/test_plays:halt_test_play",
        "//software/ai/hl/stp/play/test_plays:move_test_play",
        "//software/test_util",
        "//software/util/generic_factory",
    ],
)
