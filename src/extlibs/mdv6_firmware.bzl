load("@bazel_embedded//tools/openocd:defs.bzl", "openocd_flash")

cc_binary(
    name = "mdv6_firmware_main",
    # Linker file informs the toolchain how much RAM and flash you have as well as its locations on the chip
    additional_linker_inputs = [
        "STM32CubeIDE/STM32F031C6TX_FLASH.ld",
    ],
    linkopts = ["-T $(location STM32CubeIDE/STM32F031C6TX_FLASH.ld)"],
    target_compatible_with = [
        "@platforms//cpu:armv6-m",
        "@platforms//os:none",
    ],
    deps = [
        ":mdv6_firmware",
    ],
)

openocd_flash(
    name = "mdv6_firmware_flash",
    device_configs = [
        # Part of the STM32F0 family
        # https://www.st.com/resource/en/reference_manual/rm0091-stm32f0x1stm32f0x2stm32f0x8-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
        "target/stm32f0x.cfg",
    ],
    # WARNING! `flash_offset` attribute is broken in the Bazel rule. The .elf file (mdv6_firmware_main.stripped)
    # contains the memory mappings already, so this attribute is necessary.
    flash_offset = "",
    # .stripped strips debug symbols to reduce the size of the binary.
    # see: https://bazel.build/reference/be/c-cpp#cc_binary
    image = ":mdv6_firmware_main.stripped",
    interface_configs = [
        "interface/stlink.cfg",  # The ST-Link V2 programmer is used to flash the firmware.
    ],
)

cc_library(
    name = "mdv6_firmware",
    srcs = glob([
        "Core/Src/**/*.c",
        "*.c",
        "Src/**/*.h",
        "Drivers/STM32F0xx_HAL_Driver/Src/*_ll_*.c",
        "Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c",
        "Drivers/CMSIS/Device/ST/STM32F0xx/Source/Templates/system_stm32f0xx.c",
    ]),
    hdrs = glob([
        "Core/Inc/**/*.h",
        "*.h",
        "Inc/**/*.h",
        "Drivers/STM32F0xx_HAL_Driver/Inc/**/*.h",
        "Drivers/CMSIS/Device/ST/STM32F0xx/Include/**/*.h",
        "Drivers/CMSIS/Include/**/*.h",
    ]),
    includes = [
        "Core/Inc",
        "Drivers/STM32F0xx_HAL_Driver/Inc",
        "Drivers/CMSIS/Device/ST/STM32F0xx/Include",
        "Drivers/CMSIS/Include",
        "Inc",
    ],
    defines = [
        # Our MCU is the STM32F0251: https://www.st.com/resource/en/datasheet/stspin32f0251.pdf
        "STM32F031x6",
        # Use the Low-Layer APIs because it gives us more control over the hardware (as opposed to the HAL).
        # https://www.st.com/content/ccc/resource/technical/document/user_manual/56/32/53/cb/69/86/49/0e/DM00223149.pdf/files/DM00223149.pdf/jcr:content/translations/en.DM00223149.pdf
        "USE_FULL_LL_DRIVER",
    ],
)
