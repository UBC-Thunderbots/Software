package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ai",
    srcs = ["ai.cpp"],
    hdrs = ["ai.h"],
    deps = [
        "//software/ai/hl/stp",
        "//software/ai/hl/stp:play_info",
        "//software/ai/hl/stp/play:halt_play",
        "//software/ai/navigator",
        "//software/ai/navigator/path_planning_navigator",
        "//software/ai/primitive",
        "//software/ai/world",
        "//software/util/time:timestamp",
    ],
)

cc_library(
    name = "ai_wrapper",
    srcs = ["ai_wrapper.cpp"],
    hdrs = ["ai_wrapper.h"],
    deps = [
        "//software:typedefs",
        "//software/ai",
        "//software/ai/world",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/util/canvas_messenger",
        "//software/util/parameter:dynamic_parameters",
        "@boost//:bind",
    ],
)

cc_test(
    name = "game_state_play_selection_test",
    srcs = ["game_state_play_selection_test.cpp"],
    deps = [
        "//software/ai",
        "//software/ai/world",
        "//software/test/test_util",
        "@gtest//:gtest_main",
    ],
)
