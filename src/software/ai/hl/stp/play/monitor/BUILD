package(default_visibility = ["//visibility:public"])

cc_library(
    name = "play_monitor",
    srcs = [
        "play_intent.cpp",
        "play_monitor.cpp",
    ],
    hdrs = [
        "play_intent.h",
        "play_monitor.h",
    ],
    deps = [
        "//shared:constants",
        "//software/ai/evaluation:possession",
        "//software/geom/algorithms",
        "//software/logger",
    ],
    alwayslink = True,
)
