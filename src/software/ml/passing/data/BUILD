package(default_visibility = ["//visibility:public"])

py_library(
    name = "pass_result",
    srcs = ["pass_result.py"],
    deps = [
        "//software/evaluation/logs:event_log",
        "//software/evaluation/logs:pass_log",
        "//software/ml/data_cleanup:result_interface",
        "//software/ml/data_cleanup:stats_result",
    ],
)

py_library(
    name = "labelled_passes",
    srcs = ["labelled_passes.py"],
    deps = [
        ":pass_result",
        "//software/evaluation/logs:pass_log",
        "//software/ml/data_cleanup:stats_result",
    ],
)

py_library(
    name = "types",
    srcs = ["types.py"],
    deps = [
        ":labelled_passes",
    ],
)

py_library(
    name = "build_graph",
    srcs = ["build_graph.py"],
    deps = [
        ":labelled_passes",
        ":types",
    ],
)

py_binary(
    name = "generate_labelled_passes",
    srcs = ["generate_labelled_passes.py"],
    deps = [
        "//software/evaluation/logs:event_log",
        "//software/evaluation/logs:pass_log",
        ":labelled_passes",
        ":pass_result"
    ],
)
