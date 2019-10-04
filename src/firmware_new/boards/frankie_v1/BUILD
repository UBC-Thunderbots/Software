load("//firmware_new:rules.bzl", "cc_stm32h7_hal_library")

#cc_stm32_h7_binary(
#    name = "frankie_v1",
#    srcs = glob([
#        "*.c",
#        "*.h",
#    ]),
#    hal_config_files = glob(["*.h"]),
#)

#cc_stm32h7_binary(
#    name = "frankie_v1_main",
#    deps = [
#        ":frankie_v1_hal",
#    ],
#)

# TODO: do we need custom flags for this?
# TODO: how to enforce compiler???
# TODO: probably want a custom rule, or at least a macro for this
cc_binary(
    name = "frankie_v1_main",
    srcs = [
        "freertos.c",
        "main.c",
        "main.h",
        "stm32h7xx_hal_msp.c",
        "stm32h7xx_it.c",
        "stm32h7xx_it.h",
        "system_stm32h7xx.c",
    ],
    copts = [
        "-DSTM32H743xx",
    ],
    deps = [
        ":frankie_v1_hal",
    ],
)

# TODO: rename this, it encompasses middlewares as well now
cc_stm32h7_hal_library(
    name = "frankie_v1_hal",
    drivers_and_middleware_hdrs = [
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h743xx.h",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h",
        "Drivers/CMSIS/Include/cmsis_version.h",
        "Drivers/CMSIS/Include/cmsis_compiler.h",
        "Drivers/CMSIS/Include/mpu_armv7.h",
        "Drivers/CMSIS/Include/cmsis_gcc.h",
        "Drivers/CMSIS/Include/core_cm7.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_cortex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_def.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_exti.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_flash_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_gpio_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_hsem.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_i2c_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_mdma.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_pwr_ex.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc.h",
        "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_rcc_ex.h",
        "Drivers/CMSIS/Device/ST/STM32H7xx/Include/system_stm32h7xx.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/projdefs.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/portable.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/mpu_wrappers.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/croutine.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/event_groups.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/list.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/queue.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/stream_buffer.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/timers.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/task.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/stack_macros.h",
        "Middlewares/Third_Party/FreeRTOS/Source/include/deprecated_definitions.h",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h",
        "Drivers/CMSIS/RTOS2/Template/cmsis_os.h",
        "Drivers/CMSIS/Include/cmsis_gcc.h",
        "Drivers/CMSIS/RTOS2/Include/cmsis_os2.h",
    ],
    drivers_and_middleware_srcs = [
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c",
        "Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c",
        "Middlewares/Third_Party/FreeRTOS/Source/croutine.c",
        "Middlewares/Third_Party/FreeRTOS/Source/event_groups.c",
        "Middlewares/Third_Party/FreeRTOS/Source/list.c",
        "Middlewares/Third_Party/FreeRTOS/Source/queue.c",
        "Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c",
        "Middlewares/Third_Party/FreeRTOS/Source/tasks.c",
        "Middlewares/Third_Party/FreeRTOS/Source/timers.c",
        #        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c",
        "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c",
        "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c",
    ],
    free_rtos_config_hdrs = [
        "FreeRTOSConfig.h",
    ],
    hal_config_hdrs = [
        "stm32h7xx_hal_conf.h",
    ],
    processor = "STM32H743xx",
)
