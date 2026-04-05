package(default_visibility = ["//visibility:public"])

py_library(
    name = "train",
    srcs = ["train.py"],
    deps = [
        "//software/evaluation/logs:event_log",
        "//software/evaluation/logs:pass_log",
        "//software/ml/models/hetero_gnn",
        "//software/ml/models/hetero_gnn:config",
        "//software/ml/passing/data:build_graph",
        "//software/ml/passing/data:labelled_passes",
        "//software/ml/passing/data:pass_result",
        "//software/ml/passing/data:types",
    ],
)
