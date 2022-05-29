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
        "//software/jetson_nano:setup_nano",
        "//software/jetson_nano:thunderloop_main",
        "//software/jetson_nano/broadcasts:robot_broadcast_receiver",
        "//software/jetson_nano/broadcasts:robot_broadcast_sender_zip",
        "//software/jetson_nano/display:robot_ui_zip",
        "//software/jetson_nano/redis",
        "//software/jetson_nano/systemd:systemd_files",
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
