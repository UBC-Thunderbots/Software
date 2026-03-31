package(default_visibility = ["//visibility:public"])

py_library(
    name = "pass_samples",
    srcs = [
        "pass_event.py",
        "pass_samples.py",
    ],
    deps = [
        "//software/thunderscope:thread_safe_buffer",
        "//software/thunderscope/log/trackers:tracker",
    ],
)

py_library(
    name = "pass_results",
    srcs = [
        "pass_event.py",
        "pass_results.py",
        "pass_result_tracker.py"
    ],
    deps = [
        "//software/thunderscope/log/trackers:tracker",
    ],
)
