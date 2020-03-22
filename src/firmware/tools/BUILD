sh_binary(
    name = "legacy_flash_firmware",
    srcs = ["legacy_flash_firmware.sh"],
    data = [
        "//firmware/boards/legacy_dongle_stm32f4:bin",
        "//firmware/boards/legacy_robot_stm32f4:bin",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)
