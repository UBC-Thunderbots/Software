load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "fs_stats",
    srcs = ["fs_stats.py"],
    deps = [],
)

py_library(
    name = "pass_features",
    srcs = ["pass_features.py"],
    deps = [],
)

py_library(
    name = "stats",
    srcs = ["stats.py"],
    deps = [
        ":fs_stats",
        ":pass_features",
        "//software/thunderscope:thread_safe_buffer",
    ],
)
