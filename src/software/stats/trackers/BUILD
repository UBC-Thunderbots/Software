load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "tracker",
    srcs = [
        "__init__.py",
        "goalie_tracker.py",
        "kick_tracker.py",
        "possession_tracker.py",
        "referee_tracker.py",
        "tracker.py",
        "tracker_builder.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//software/stats/logs",
        "//software/thunderscope:time_provider",
    ],
)
