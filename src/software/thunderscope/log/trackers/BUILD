load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "tracker",
    srcs = ["tracker.py"],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
    ],
)

py_library(
    name = "goalie_tracker",
    srcs = ["goalie_tracker.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        ":tracker",
    ],
)

py_library(
    name = "kick_tracker",
    srcs = ["kick_tracker.py"],
    deps = [
        ":tracker",
    ],
)

py_library(
    name = "possession_tracker",
    srcs = ["possession_tracker.py"],
    deps = [
        ":tracker",
    ],
)

py_library(
    name = "referee_tracker",
    srcs = ["referee_tracker.py"],
    deps = [
        ":tracker",
    ],
)

py_library(
    name = "tracker_builder",
    srcs = ["tracker_builder.py"],
    deps = [
        ":tracker",
    ],
)

py_library(
    name = "tracker_module",
    srcs = [
        "__init__.py",
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
