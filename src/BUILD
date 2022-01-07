load("@rules_python//python:defs.bzl", "py_runtime")
load("@rules_python//python:defs.bzl", "py_runtime_pair")

# targets for hermetic Python runtime
py_runtime(
    name = "python3_runtime",
    files = ["@python_interpreter//:files"],
    interpreter = "@python_interpreter//:python_bin",
    python_version = "PY3",
    visibility = ["//visibility:public"],
)

py_runtime_pair(
    name = "py_runtime_pair",
    py2_runtime = None,
    py3_runtime = ":python3_runtime",
)

toolchain(
    name = "py_toolchain",
    toolchain = ":py_runtime_pair",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)

load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")

# Generates compile_commands.json
refresh_compile_commands(
    name = "refresh_compile_commands",
    targets = [
        "//software/...",
        "//proto/...",
        "//extlibs/...",
        "//shared/...",
    ],
)
