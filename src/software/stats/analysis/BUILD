load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "metrics",
    srcs = ["metrics.py"],
    deps = [
        "//proto:import_all_protos",
        "//software/stats/logs:event_log",
    ],
)

py_binary(
    name = "analysis",
    srcs = ["analysis.py"],
    deps = [
        ":metrics",
        "//software/stats/logs:event_log",
        "//software/thunderscope:constants",
    ],
)
