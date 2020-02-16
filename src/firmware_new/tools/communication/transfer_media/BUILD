package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transfer_medium",
    hdrs = ["transfer_medium.h"],
)

cc_library(
    name = "network_medium",
    srcs = ["network_medium.cpp"],
    hdrs = ["network_medium.h"],
    restricted_to = ["//cc_toolchain:k8"],
    deps = [
        ":transfer_medium",
        "//software/multithreading:thread_safe_buffer",
        "@g3log",
    ],
)
