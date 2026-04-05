package(default_visibility = ["//visibility:public"])

py_library(
    name = "log_interface",
    srcs = ["log_interface.py"],
    deps = [
        "//software/thunderscope:time_provider",
    ],
)

py_library(
    name = "world_state_log",
    srcs = ["world_state_log.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        ":log_interface",
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
    name = "pass_log",
    srcs = ["pass_log.py"],
    deps = [
        ":event_log",
        ":log_interface",
    ],
)

py_library(
    name = "logs",
    deps = [
        ":event_log",
        ":log_interface",
        ":pass_log",
        ":world_state_log",
    ],
)
