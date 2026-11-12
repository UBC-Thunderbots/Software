load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "stats_logger",
    srcs = ["stats_logger.py"],
    deps = [
        "//software/stats/trackers:tracker",
    ],
)
