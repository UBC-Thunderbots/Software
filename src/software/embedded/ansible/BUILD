package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@ansible_deps//:requirements.bzl", "requirement")
load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

py_binary(
    name = "run_ansible",
    srcs = [
        "run_ansible.py",
    ],
    data = [
        ":playbooks",
        ":scripts",
        "//software/embedded:setup_robot_software_deps",
        "//software/embedded:thunderloop_main",
        "//software/embedded/linux_configs/jetson_nano:jetson_nano_files",
        "//software/embedded/linux_configs/pi:pi_files",
        "//software/embedded/linux_configs/systemd:systemd_files",
        "//software/embedded/redis",
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
    name = "scripts",
    data = glob([
        "scripts/**/*.py",
    ]),
)
