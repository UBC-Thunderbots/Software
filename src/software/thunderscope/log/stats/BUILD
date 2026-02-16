load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "fullsystem_stats",
    srcs = ["fullsystem_stats.py"],
    deps = [
        "//software/thunderscope/log/trackers:tracker_module"
    ],
)

py_library(
    name = "stats",
    srcs = ["stats.py"],
    deps = [
        ":fullsystem_stats",
        "//software/thunderscope:thread_safe_buffer",
    ],
)
