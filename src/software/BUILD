package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "full_system",
    srcs = ["full_system_main.cpp"],
    deps = [
        "//software:constants",
        "//software/ai:ai_wrapper",
        "//software/ai/hl/stp:play_info",
        "//software/backend:backend_factory",
        "//software/backend:grsim_backend",
        "//software/backend:radio_backend",
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/visualizer:visualizer_wrapper",
        "@boost//:program_options",
        "@g3log",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
)
