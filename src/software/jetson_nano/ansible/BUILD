package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@ansible_deps//:requirements.bzl", "requirement")

py_binary(
    name = "dynamic_inventory",
    srcs = ["dynamic_inventory.py"],
    data = ["playbooks/example_playbook.yml"],
    deps = [
        requirement("ansible"),
    ],
)

py_binary(
    name = "run_ansible",
    srcs = ["run_ansible.py"],
    data = ["playbooks/example_playbook.yml"],
    deps = [
        requirement("ansible"),
    ],
)

#genrule(
#    name = "setup",
#    outs = ["example_playbook.yml"],
#)
#
#genrule(
#    name = "setup",
#    outs = ["ansible_script.sh"],
#    cmd_bash = "echo ansible >> ansible.sh && chmod +x ansible.sh",
#)

py_binary(
    name = "ansible_py",
    srcs = [],
    deps = [requirement("ansible")],
)

sh_binary(
    name = "ansible_sh",
    srcs = ["ansible_sh.sh"],
    args = ["$(location :requirements(\"ansible\")"],
    data = [
        requirement("ansible"),
    ],
)
