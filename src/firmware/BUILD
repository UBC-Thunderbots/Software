package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "mdv6_firmware_main",
    srcs = ["main.c"],
    deps = [
        ":mdv6_firmware",
        ":mdv6_main_hooks",
    ],
)

cc_library(
    name = "mc_config",
    srcs = ["mc_config.c"],
    hdrs = ["mc_config.h"],
    deps = [
        ":mdv6_main_hooks",
        "//firmware/mcsdk:pid_regulator",
    ],
)

cc_library(
    name = "mdv6_main_hooks",
    hdrs = ["main.h"],
    deps = [
        "//firmware/stm32f0xx:stm32f0xx",
    ],
)

cc_library(
    name = "mdv6_firmware",
    srcs = [
        "mc_api.c",
        "mc_app_hooks.c",
        "mc_configuration_registers.c",
        "mc_interface.c",
        "mc_math.c",
        "mc_parameters.c",
        "mc_tasks.c",
        "pwm_curr_fdbk.c",
        "r1_ps_pwm_curr_fdbk.c",
        "register_interface.c",
        "regular_conversion_manager.c",
        "stm32f0xx_hal_msp.c",
        "stm32f0xx_it.c",
        "stm32f0xx_mc_it.c",
    ],
    hdrs = [
     "mc_api.h",
     "mc_app_hooks.h",
     "mc_configuration_registers.h",
     "mc_interface.h",
     "mc_math.h",
     "mc_parameters.h",
     "mc_tasks.h",
     "parameters_conversion.h",
     "parameters_conversion_f0xx.h",
     "pmsm_motor_parameters.h",
     "power_stage_parameters.h",
     "r1_ps_pwm_curr_fdbk.h",
     "register_interface.h",
     "regular_conversion_manager.h",
     "stm32f0xx_it.h",
    ],
    deps = [
        "//firmware/mcsdk:mc_type",
        "//firmware/mcsdk:mcsdk",
        "//firmware/motorcontrol:motorcontrol",
        "//firmware/stm32f0xx:stm32f0xx",
    ],
)
