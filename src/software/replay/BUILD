package(default_visibility = ["//visibility:public"])

cc_library(
    name = "replay_logger",
    srcs = ["replay_logger.cpp"],
    hdrs = [
        "replay_logger.h",
    ],
    deps = [
        "//software/geom",
        "//software/proto:tbots_cc_proto",
        "//software/time:timestamp",
        "//software/multithreading:threaded_observer",
        "//software/world",
    ],
)
