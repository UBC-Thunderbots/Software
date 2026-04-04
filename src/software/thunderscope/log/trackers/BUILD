load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "tracker",
    srcs = [
        "__init__.py",
        "goalie_tracker.py",
        "kick_tracker.py",
        "possession_tracker.py",
        "referee_tracker.py",
        "tracked_event.py",
        "tracker.py",
        "tracker_builder.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//software/thunderscope:time_provider",
    ],
)
