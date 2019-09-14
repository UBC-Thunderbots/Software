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
