package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@robot_diagnostics_cli_deps//:requirements.bzl", "requirement")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
load("@rules_pkg//:pkg.bzl", "pkg_tar")
load("//:defs.bzl", "pkg_executable")

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
        "//proto:import_all_protos",
        "//software:py_constants.so",
        "//software:python_bindings.so",
        "//software/embedded/constants:py_constants",
        "//software/thunderscope:thread_safe_buffer",
    ],
    deps = [
        "//software/embedded/robot_diagnostics_cli:embedded_communication",
        "//software/embedded/robot_diagnostics_cli:embedded_data",
        requirement("typer_shell"),
        requirement("rich"),
        requirement("protobuf"),
        requirement("inquirerpy"),
        requirement("pyyaml")
    ],
)

pkg_tar(
    name = "robot_diagnostics_cli_tar",
    srcs = [
        ":robot_diagnostics_cli_pkg",
    ],
    extension = "tar.gz",
)

pkg_executable(
    name = "robot_diagnostics_cli_pkg",
    bin = ":robot_diagnostics_cli",
    path = "robot_diagnostics_cli",
)

py_library(
    name = "embedded_communication",
    srcs = [":embedded_communication.py"],
    deps = [
        requirement("protobuf"),
        requirement("pyyaml")
    ],
)

py_library(
    name = "embedded_data",
    srcs = [":embedded_data.py"],
    deps = [
        requirement("pyyaml")
    ],
)
