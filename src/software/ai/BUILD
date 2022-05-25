package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ai",
    srcs = [
        "ai.cpp",
        "play_selection_fsm.cpp",
    ],
    hdrs = [
        "ai.h",
        "play_selection_fsm.h",
    ],
    # We include all the plays here so that the static
    # variables in the .cpp files that auto-register plays with the
    # PlayFactory. This addresses the issue explained here:
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    deps = [
        "//proto:play_info_msg_cc_proto",
        "//software/ai/hl/stp/play:all_plays",
        "//software/ai/hl/stp/play:assigned_tactics_play",
        "//software/ai/hl/stp/play:halt_play",
        "//software/ai/hl/stp/play:play_factory",
        "//software/ai/hl/stp/tactic:tactic_factory",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_library(
    name = "threaded_ai",
    srcs = ["threaded_ai.cpp"],
    hdrs = ["threaded_ai.h"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared/parameter:cpp_configs",
        "//software/ai",
        "//software/gui/drawing:draw_functions",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/world",
        "@boost//:bind",
    ],
)
