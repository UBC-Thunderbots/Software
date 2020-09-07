cc_library(
    name = "network_logger",
    srcs = [
        "network_logger.c",
    ],
    hdrs = [
        "network_logger.h",
    ],
    restricted_to = ["//cc_toolchain:stm32h7"],
    deps = [
        "//firmware_new/boards/frankie_v1:frankie_v1_lwip",
        "//firmware_new/boards/frankie_v1:hal",
        "//firmware_new/boards/frankie_v1:lwip",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
)
