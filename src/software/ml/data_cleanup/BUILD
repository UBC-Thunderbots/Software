package(default_visibility = ["//visibility:public"])

py_library(
    name = "stats_result",
    srcs = ["stats_result.py"],
    data = [
        "//software:python_bindings.so",
    ],
    deps = [
        ":result_interface",
        "//software/evaluation/logs:event_log",
        "//software/evaluation/logs:log_interface",
    ],
)

py_library(
    name = "result_interface",
    srcs = ["result_interface.py"],
    deps = [
        "//software/evaluation/logs:log_interface",
    ],
)
