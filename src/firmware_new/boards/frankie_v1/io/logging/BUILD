package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    srcs = ["logger.c"],
    hdrs = ["logger.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware_new/boards/frankie_v1:frankie_v1_lwip",
        "//firmware_new/boards/frankie_v1:hal",
        "//firmware_new/boards/frankie_v1:lwip",
        "//firmware_new/boards/frankie_v1:rtos",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
)

cc_library(
    name = "log_sink",
    srcs = ["log_sink.c"],
    hdrs = ["log_sink.h"],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware_new/boards/frankie_v1:frankie_v1_lwip",
        "//firmware_new/boards/frankie_v1:hal",
        "//firmware_new/boards/frankie_v1:lwip",
        "//firmware_new/boards/frankie_v1:rtos",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
)
