load("@rules_cc//cc:defs.bzl", "cc_toolchain", "cc_toolchain_suite")

package(default_visibility = ["//visibility:public"])

# Create environments for each CPU this toolchain supports
environment(name = "k8")

environment(name = "k8_aarch64_linux")

environment(name = "aarch64")

environment_group(
    name = "cpus",
    defaults = [
        ":aarch64",
        ":k8",
        ":k8_aarch64_linux",
    ],
    environments = [
        ":aarch64",
        ":k8",
        ":k8_aarch64_linux",
    ],
)

config_setting(
    name = "cpu_aarch64",
    values = {"cpu": "aarch64"},
)

config_setting(
    name = "cpu_k8",
    values = {"cpu": "k8"},
)

config_setting(
    name = "cpu_k8_aarch64_linux",
    values = {"cpu": "k8_aarch64_linux"},
)

# Create a constraint for the glibc version to target the correct glibc version for the robot
constraint_setting(name = "glibc_version")

constraint_value(
    name = "glibc_2_28",
    constraint_setting = ":glibc_version",
)

platform(
    name = "robot",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
        ":glibc_2_28",
    ],
)

platform(
    name = "motor_board",
    constraint_values = [
        "@platforms//cpu:armv6-m",
        "@platforms//os:none",
    ],
)

# This represents a mapping of CPU -> Compiler To Use
cc_toolchain_suite(
    name = "toolchain",
    toolchains = {
        # k8 is any x86 system
        "k8": "cc_toolchain_linux_k8_gcc",
        "k8|compiler": "cc_toolchain_linux_k8_gcc",
        # aarch64 is for ARM64 systems and the robot
        "aarch64": "cc_toolchain_linux_aarch64_gcc",
        "aarch64|compiler": "cc_toolchain_linux_aarch64_gcc",
        # cross compile on x86 for arm64 (the robot)
        "k8_aarch64_linux": "cc_toolchain_k8_aarch64_linux",
        "k8_aarch64_linux|compiler": "cc_toolchain_k8_aarch64_linux",
    },
)
