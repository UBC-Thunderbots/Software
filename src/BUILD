load("@rules_python//python:defs.bzl", "py_runtime")
load("@rules_python//python:defs.bzl", "py_runtime_pair")
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
