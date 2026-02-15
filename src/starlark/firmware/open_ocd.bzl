load("@bazel_embedded//tools/openocd:defs.bzl", "openocd_flash")

package(default_visibility = ["//visibility:public"])

def _openocd_extension_impl(_ctx):
    openocd_deps()

openocd_extension = module_extension(
    implementation = _openocd_extension_impl,
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
