package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stm32f0xx",
    hdrs = [
        "stm32f031x6.h",
        "stm32f0xx.h",
        "stm32f0xx_hal.h",
        "stm32f0xx_hal_conf.h",
        "stm32f0xx_hal_def.h",
        "stm32f0xx_hal_rcc.h",
        "stm32f0xx_ll_bus.h",
        "stm32f0xx_ll_rcc.h",
        "system_stm32f0xx.h",
    ],
    srcs = [
        "system_stm32f0xx.c",
    ],
    defines = [
        # Our MCU is the STM32F0251: https://www.st.com/resource/en/datasheet/stspin32f0251.pdf
        "STM32F031x6",
    ],
    deps = [
        "//firmware/cmsis:cmsis",
    ],
)
