load("@ansible_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")
load("@rules_python//python:pip.bzl", "compile_pip_requirements")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = ":requirements.in",
    requirements_txt = ":requirements_lock.txt",
)

py_binary(
    name = "run_ansible",
    srcs = [
        "run_ansible.py",
    ],
    data = [
        ":playbooks",
        ":tasks",
        "//software/embedded:setup_robot_software_deps",
        "//software/embedded:thunderloop_main",
        "//software/embedded/linux_configs/jetson_nano:jetson_nano_files",
        "//software/embedded/linux_configs/pi:pi_files",
        "//software/embedded/linux_configs/systemd:systemd_files",
        "//software/embedded/redis",
        "//software/embedded/robot_diagnostics_cli:robot_diagnostics_cli_tar",
        "//software/embedded/services:robot_auto_test",
        "//software/power:powerloop_tar",
    ],
    deps = [
        requirement("ansible"),
    ],
)

filegroup(
    name = "playbooks",
    data = glob([
        "playbooks/**/*.yml",
    ]),
)

filegroup(
    name = "tasks",
    data = glob([
        "tasks/**/*.yml",
    ]),
)
