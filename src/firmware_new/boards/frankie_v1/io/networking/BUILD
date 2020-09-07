package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_multicast_communication",
    srcs = [
        "proto_multicast_communication_profile.c",
        "proto_multicast_communication_tasks.c",
    ],
    hdrs = [
        "proto_multicast_communication_profile.h",
        "proto_multicast_communication_tasks.h",
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
