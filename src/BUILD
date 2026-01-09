load("@hedron_compile_commands//:refresh_compile_commands.bzl", "refresh_compile_commands")
load("@rules_python//python:defs.bzl", "py_runtime", "py_runtime_pair")

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
