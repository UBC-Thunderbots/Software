load("@protobuf//:protobuf_deps.bzl", "protobuf_deps")
load("@robot_diagnostics_cli_deps//:requirements.bzl", "requirement")
load("@rules_pkg//:pkg.bzl", "pkg_tar")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("//:defs.bzl", "pkg_executable")

package(default_visibility = ["//visibility:public"])

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
    ],
)

py_library(
    name = "embedded_data",
    srcs = [":embedded_data.py"],
)
