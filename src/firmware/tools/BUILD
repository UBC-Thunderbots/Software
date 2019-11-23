sh_binary(
    name = "flash_firmware",
    srcs = ["flash_firmware.sh"],
    data = [
        "//firmware:firmware_old_main_bin",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)
