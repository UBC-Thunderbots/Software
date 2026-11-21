load("@arm_none_eabi//toolchain:toolchain.bzl", "arm_none_eabi_toolchain")

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
        "@platforms//cpu:armv6-m",
        "@platforms//os:none",
    ],
)
