load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "fs_stats",
    srcs = ["fs_stats.py"],
    deps = [],
)

py_library(
    name = "stats",
    srcs = ["stats.py"],
    deps = [
        ":fs_stats",
        "//software/thunderscope:thread_safe_buffer",
    ],
)
