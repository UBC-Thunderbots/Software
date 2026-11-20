load("@bazel_embedded//tools/openocd:defs.bzl", "openocd_flash")
load("@rules_cc//cc:cc_binary.bzl", "cc_binary")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_binary(
    name = "mdv6.5_firmware_main",
    # Linker file informs the toolchain how much RAM and flash you have as well as its locations on the chip
    additional_linker_inputs = [
        "Official_MD65_Proj/STM32CubeIDE/STM32G431VBTX_FLASH.ld",
    ],
    linkopts = ["-T $(location Official_MD65_Proj/STM32CubeIDE/STM32G431VBTX_FLASH.ld)"],
    target_compatible_with = [
        "@platforms//cpu:armv7e-m",
        "@platforms//os:none",
    ],
    deps = [
        ":mdv6.5_firmware",
    ],
)

openocd_flash(
    name = "mdv6.5_firmware_flash",
    device_configs = [
        # Part of the STM32G4 family
        # https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
        "target/stm32g4x.cfg",
    ],
    # WARNING! `flash_offset` attribute is broken in the Bazel rule. The .elf file (mdv6.5_firmware_main.stripped)
    # contains the memory mappings already, so this attribute is necessary.
    flash_offset = "",
    # .stripped strips debug symbols to reduce the size of the binary.
    # see: https://bazel.build/reference/be/c-cpp#cc_binary
    image = ":mdv6.5_firmware_main.stripped",
    interface_configs = [
        "interface/stlink.cfg",  # The ST-Link V2 programmer is used to flash the firmware.
    ],
)

cc_library(
    name = "mdv6.5_firmware",
    # Same source set as the STM32CubeIDE project (STM32CubeIDE/.project).
    srcs = glob([
        "Official_MD65_Proj/Src/*.c",
        "Official_MD65_Proj/STM32CubeIDE/Application/Startup/*.s",
        "Official_MD65_Proj/STM32CubeIDE/Application/User/*.c",
    ]) + [
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_comp.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cordic.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dac_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_i2c_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_opamp_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/bus_voltage_sensor.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/circle_limitation.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/digital_output.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/enc_align_ctrl.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/encoder_speed_pos_fdbk.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/feed_forward_ctrl.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/ntc_temperature_sensor.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/open_loop.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/pid_regulator.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/pqd_motor_power_measurement.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/ramp_ext_mngr.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/r_divider_bus_voltage_sensor.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/revup_ctrl.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/speed_pos_fdbk.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Src/virtual_speed_sensor.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/r3_2_g4xx_pwm_curr_fdbk.c",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Src/stspin32g4.c",
    ],
    hdrs = glob([
        "Official_MD65_Proj/Inc/**/*.h",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Inc/**/*.h",
        "Official_MD65_Proj/Drivers/CMSIS/Device/ST/STM32G4xx/Include/**/*.h",
        "Official_MD65_Proj/Drivers/CMSIS/Include/**/*.h",
        "Official_MD65_Proj/Drivers/CMSIS/DSP/Include/**/*.h",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/**/*.h",
    ]),
    includes = [
        "Official_MD65_Proj/Inc",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Inc",
        "Official_MD65_Proj/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
        "Official_MD65_Proj/Drivers/CMSIS/Device/ST/STM32G4xx/Include",
        "Official_MD65_Proj/Drivers/CMSIS/Include",
        "Official_MD65_Proj/Drivers/CMSIS/DSP/Include",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/Any/Inc",
        "Official_MD65_Proj/MCSDK_v6.4.1-Full/MotorControl/MCSDK/MCLib/G4xx/Inc",
    ],
    defines = [
        # Our MCU is the STM32G431VB (STSPIN32G4).
        "STM32G431xx",
        "USE_HAL_DRIVER",
        "ARM_MATH_CM4",
    ],
    alwayslink = True,
)
