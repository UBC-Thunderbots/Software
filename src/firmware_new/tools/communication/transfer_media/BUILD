package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transfer_medium",
    hdrs = ["transfer_medium.h"],
)

cc_library(
    name = "network_medium",
    srcs = ["network_medium.cpp"],
    hdrs = ["network_medium.h"],
    deps = [
        ":transfer_medium",
        "@g3log",
        "//software/multithreading:thread_safe_buffer",
    ],
)
