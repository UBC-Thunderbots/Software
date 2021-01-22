package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "full_system",
    srcs = ["full_system_main.cpp"],
    deps = [
        "//software:constants",
        "//software/ai:threaded_ai",
        "//software/ai/hl/stp:play_info",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/gui/full_system:threaded_full_system_gui",
        "//software/logger",
        "//software/multithreading:observer_subject_adapter",
        "//software/parameter:dynamic_parameters",
        "//software/proto/logging:proto_logger",
        "//software/proto/message_translation:ssl_wrapper",
        "//software/sensor_fusion:threaded_sensor_fusion",
        "//software/util/design_patterns:generic_factory",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
)
