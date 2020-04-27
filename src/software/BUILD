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
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "//software/visualizer:visualizer_wrapper",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
)
