package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@ansible_deps//:requirements.bzl", "requirement")

py_binary(
    name = "run_ansible",
    srcs = ["run_ansible.py"],
    data = [":playbooks"],
    deps = [
        requirement("ansible"),
        "//software/jetson_nano/broadcasts:robot_broadcast_receiver",
    ],
)

filegroup(
    name = "playbooks",
    data = glob([
        "playbooks/**/*.yml",
    ]),
)
