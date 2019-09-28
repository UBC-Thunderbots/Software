package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ai",
    srcs = ["ai.cpp"],
    hdrs = ["ai.h"],
    # We include all the plays here so that that the static
    # variables in the .cpp files that auto-register plays with the
    # PlayFactory. This addresses the issue explained here:
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    deps = [
        "//software/ai/hl/stp",
        "//software/ai/hl/stp:play_info",
        "//software/ai/hl/stp/play:all_plays",
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
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
