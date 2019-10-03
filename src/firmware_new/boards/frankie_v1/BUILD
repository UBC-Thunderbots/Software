load("//firmware_new:rules.bzl", "cc_stm32_h7_binary")

cc_stm32_h7_binary(
    name = "frankie_v1",
    srcs = glob([
        "*.c",
        "*.h",
    ]),
    hal_config_files = glob(["*.h"]),
)
