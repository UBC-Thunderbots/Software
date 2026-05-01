load("@ml_deps//:requirements.bzl", "requirement")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = ":requirements.in",
    requirements_txt = ":requirements_lock.txt",
)

py_binary(
    name = "train_passing",
    srcs = ["passing/train_passing.py"],
    deps = [
        requirement("torch"),
        requirement("pandas"),
        requirement("numpy"),
        requirement("torch_geometric"),
        requirement("scikit-learn"),
        requirement("onnxscript"),
        "//software/evaluation/logs:event_log",
        "//software/evaluation/logs:pass_log",
        "//software/ml/models/hetero_gnn",
        "//software/ml/models/hetero_gnn:config",
        "//software/ml/passing/data:build_graph",
        "//software/ml/passing/data:labelled_passes",
        "//software/ml/passing/data:types",
    ],
)
