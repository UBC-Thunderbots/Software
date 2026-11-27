load("@arm_none_eabi//toolchain:toolchain.bzl", "arm_none_eabi_toolchain")

package(default_visibility = ["//visibility:public"])

platform(
    name = "robot",
    constraint_values = [
        "@platforms//os:linux",
        "@platforms//cpu:aarch64",
    ],
)

platform(
    name = "motor_board",
    constraint_values = [
        "@platforms//os:none",
        "@platforms//cpu:armv6-m",
    ],
)

arm_none_eabi_toolchain(
    name = "motor_board_toolchain",
    copts = [
        "-mcpu=cortex-m0",
        "-mthumb",
        "-O3",  # Recommended for firmware builds
        "-ffunction-sections",
        "-fdata-sections",
    ],
    linkopts = [
        "-mcpu=cortex-m0",
        "-mthumb",
        "-specs=nano.specs",
        "-specs=nosys.specs",
        "-Wl,--gc-sections",
    ],
    target_compatible_with = [
        "@platforms//os:none",
        "@platforms//cpu:armv6-m",
    ],
)
