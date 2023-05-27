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
        "//software/jetson_nano:setup_nano",
        "//software/jetson_nano:thunderloop_main",
        "//software/jetson_nano/display:robot_ui_zip",
        "//software/jetson_nano/linux_configs:device_tree",
        "//software/jetson_nano/linux_configs:extlinux",
        "//software/jetson_nano/linux_configs/systemd:systemd_files",
        "//software/jetson_nano/redis",
        #        "//software/jetson_nano/services:robot_auto_test_invoker",
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
