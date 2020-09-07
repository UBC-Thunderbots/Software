package(default_visibility = ["//visibility:public"])

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
        "//firmware_new/boards/frankie_v1/io/networking:proto_multicast_communication",
        "//firmware_new/boards/frankie_v1:frankie_v1_lwip",
        "//firmware_new/boards/frankie_v1:hal",
        "//firmware_new/boards/frankie_v1:lwip",
        "//firmware_new/boards/frankie_v1:rtos",
        "//shared/proto:tbots_nanopb_proto",
        "@nanopb",
    ],
)
