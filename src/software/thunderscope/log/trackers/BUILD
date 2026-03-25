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
        "tracker.py",
        "tracker_builder.py",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":goalie_tracker",
        ":kick_tracker",
        ":possession_tracker",
        ":referee_tracker",
        ":tracker_builder",
    ],
)
