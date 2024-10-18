package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@robot_diagnostics_cli_deps//:requirements.bzl", "requirement")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

compile_pip_requirements(
    name = "requirements",
    src = ":requirements.in",
    requirements_txt = "requirements_lock.txt",
)

py_binary(
    name = "robot_diagnostics_cli",
    srcs = [
        ":robot_diagnostics_cli.py"
    ],
    deps = [
        requirement("typer"),
        requirement("rich"),
    ],
)

