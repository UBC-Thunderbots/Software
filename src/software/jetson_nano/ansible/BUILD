package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@ansible_deps//:requirements.bzl", "requirement")

py_binary(
    name = "dynamic_inventory",
    srcs = ["dynamic_inventory.py"],
    data = ["setup.yml"],
    deps = [requirement("ansible")],
)

genrule(
    name = "setup",
    outs = ["ansible_script.sh"],
    cmd_bash = "echo ansible >> ansible.sh && chmod +x ansible.sh",
)

py_binary(
    name = "ansible_py",
    srcs = [],
    deps = [requirement("ansible")],
)

sh_binary(
    name = "ansible_sh",
    srcs = ["ansible_sh.sh"],
    data = [
        requirement("ansible"),
    ],
)
