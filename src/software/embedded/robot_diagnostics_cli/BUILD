package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@robot_diagnostics_cli_deps//:requirements.bzl", "requirement")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")

compile_pip_requirements(
    name = "requirements",
    src = ":requirements.in",
    requirements_txt = "requirements_lock.txt",
)

py_binary(
    name = "robot_diagnostics_cli",
    srcs = [
        ":robot_diagnostics_cli.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
        "//proto:import_all_protos",
    ],
    deps = [
        requirement("typer_shell"),
        requirement("rich"),
        requirement("protobuf"),
    ],
)

