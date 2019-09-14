package(default_visibility = ["//visibility:public"])

cc_library(
    name = "typedefs",
    hdrs = ["typedefs.h"],
)

cc_library(
    name = "full_system",
    srcs = ["full_system_main.cpp"],
    deps = [
        "//software/ai:ai_wrapper",
        "//software/backend:backend_factory",
        "//software/backend:grsim_backend",
        "//software/backend:radio_backend",
        "//software/util:constants",
        "//software/util/canvas_messenger",
        "//software/util/logger",
        "//software/util/parameter:dynamic_parameters",
        "@boost//:program_options",
        "@g3log//:all",
    ],
)
