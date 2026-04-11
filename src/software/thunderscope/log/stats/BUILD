package(default_visibility = ["//visibility:public"])
load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_library(
    name = "fullsystem_stats",
    srcs = ["fullsystem_stats.py"],
    deps = [
        "//software/thunderscope/log/trackers:tracker",
    ],
)

py_library(
    name = "stats",
    srcs = ["stats.py"],
    deps = [
        ":fullsystem_stats",
        "//software/thunderscope:thread_safe_buffer",
        requirement("rich"),
    ],
)
