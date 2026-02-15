package(default_visibility = ["//visibility:public"])

cc_library(
    name = "halt_test_play",
    srcs = [
        "halt_test_play.cpp",
    ],
    hdrs = [
        "halt_test_play.h",
        "halt_test_play_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/halt:halt_tactic",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_library(
    name = "move_test_play",
    srcs = ["move_test_play.cpp"],
    hdrs = ["move_test_play.h"],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)

cc_test(
    name = "halt_test_play_fsm_test",
    srcs = ["halt_test_play_fsm_test.cpp"],
    deps = [
        ":halt_test_play",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
