load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "log_interface",
    srcs = ["log_interface.py"],
    deps = [
        "//proto:import_all_protos",
        "//software/thunderscope:time_provider",
    ],
)

py_library(
    name = "type_utils",
    srcs = ["type_utils.py"],
    deps = [],
)

py_library(
    name = "world_state_log",
    srcs = ["world_state_log.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        ":log_interface",
        ":type_utils",
    ],
)

py_library(
    name = "event_log",
    srcs = ["event_log.py"],
    deps = [
        ":log_interface",
        ":world_state_log",
    ],
)

py_library(
    name = "logs",
    deps = [
        ":event_log",
        ":log_interface",
        ":world_state_log",
    ],
)
