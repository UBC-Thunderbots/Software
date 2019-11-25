package(default_visibility = ["//visibility:public"])

load("//firmware_new:rules.bzl", "stm32h7_hal_library_files_genrule")

cc_binary(
    name = "frankie_v1_main",
    srcs = ["main.c"],
    copts = [
        "-DUSE_HAL_DRIVER",
        "-DSTM32H743xx",
    ],
    linkopts = [
        "-T$(location //firmware_new/boards/frankie_v1:STM32H743ZITx_FLASH.ld)",
        "-specs=nano.specs",
        "-lc",
        "-lm",
        "-lnosys",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        ":STM32H743ZITx_FLASH.ld",
        ":frankie_v1_main_lib",
        "@nanopb//:nanopb",
        "//firmware_new/proto:fw_c_proto"
    ],
)

cc_library(
    name = "frankie_v1_main_lib",
    srcs = glob(
        [
            "*.c",
            "*.s",
        ],
        exclude = ["main.c"],
    ),
    hdrs = glob(["*.h"]),
    copts = [
        "-DSTM32H743xx",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [":hal_library"],
    # This is _very_ important. If we don't do this, bazel will not compile in the
    # interrupt handling code, which *will not* cause an error, and *will* cause
    # interrupts to stop working
    alwayslink = True,
)

stm32h7_hal_library_files_genrule("hal_library_files")

# We got some of this list of hdrs/srcs by looking at the Makefile STM32CubeMX generates,
# (if the project type is set to `MakeFile`) and others by simply attempting compilation,
# seeing where stuff failed, and finding the correct header in the downloaded files
cc_library(
    name = "hal_library",
    srcs = [
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_eth_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c",
    ],
    hdrs = [
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h743xx.h",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/system_stm32h7xx.h",
        "Drivers/CMSIS/Include/cmsis_compiler.h",
        "Drivers/CMSIS/Include/cmsis_gcc.h",
        "Drivers/CMSIS/Include/cmsis_version.h",
        "Drivers/CMSIS/Include/core_cm7.h",
        "Drivers/CMSIS/Include/mpu_armv7.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_cortex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_def.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_eth.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_eth_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_exti.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_hsem.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_mdma.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pcd.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pcd_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_tim.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_tim_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_uart.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_uart_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_usb.h",
        "stm32h7xx_hal_conf.h",
    ],
    copts = [
        "-DSTM32H743xx",
    ],
    includes = [
        "./",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/",
        "Drivers/CMSIS/Include/",
        "Drivers/CMSIS/RTOS2/Include/",
        "Drivers/CMSIS/RTOS2/Template/",
        "Drivers/STM32H7xx_HAL_Driver/Inc",
        "Middlewares/Third_Party/FreeRTOS/Source/include",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
)
