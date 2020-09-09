package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "full_system",
    srcs = ["full_system_main.cpp"],
    deps = [
        "//software:constants",
        "//software/ai:ai_wrapper",
        "//software/ai/hl/stp:play_info",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/sensor_fusion:threaded_sensor_fusion",
        "//software/util/design_patterns:generic_factory",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
)
