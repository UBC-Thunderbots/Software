load("@rules_python//python:defs.bzl", "py_runtime", "py_runtime_pair")
load("@rules_python//python/cc:py_cc_toolchain.bzl", "py_cc_toolchain")

py_runtime(
    name = "python3_runtime",
    interpreter_path = "/opt/tbotspython/bin/python3",
    python_version = "PY3",
    visibility = ["//visibility:public"],
)

py_runtime_pair(
    name = "py_runtime_pair",
    py2_runtime = None,
    py3_runtime = ":python3_runtime",
)

toolchain(
    name = "py_toolchain",
    toolchain = ":py_runtime_pair",
    toolchain_type = "@bazel_tools//tools/python:toolchain_type",
)

toolchain(
    name = "py_cc_toolchain_host_x86_64",
    exec_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    toolchain = ":py_cc_toolchain_host_data",
    toolchain_type = "@rules_python//python/cc:toolchain_type",
)

toolchain(
    name = "py_cc_toolchain_host_aarch64",
    exec_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    toolchain = ":py_cc_toolchain_host_data",
    toolchain_type = "@rules_python//python/cc:toolchain_type",
)

py_cc_toolchain(
    name = "py_cc_toolchain_host_data",
    headers = "@py_cc_toolchain_host//:headers",
    libs = ":empty",
    python_version = "3.12",
)

py_cc_toolchain(
    name = "k8_aarch64_linux_py_cc_toolchain_data",
    headers = "@py_cc_toolchain_for_k8_aarch64_linux//:headers",
    libs = ":empty",
    python_version = "3.12",
)

# We don't need this, but it's required by the py_cc_toolchain
cc_library(
    name = "empty",
    visibility = ["//visibility:private"],
)

toolchain(
    name = "k8_aarch64_linux_py_cc_toolchain",
    exec_compatible_with = [
        # Higher version of bazel has stricter hermeticy requirement,
        # thus refuses to pull in local python dev headers from aarch64.
        # We will need to set up this toolchain for both architectures.
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    toolchain = ":k8_aarch64_linux_py_cc_toolchain_data",
    toolchain_type = "@rules_python//python/cc:toolchain_type",
)
