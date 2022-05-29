package(default_visibility = ["//visibility:public"])

load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)

platformio_library(
    name = "uart_framing_platformio",
    hdr = "uart_framing.hpp",
    deps = ["//proto/message_translation:power_frame_msg_platformio"],
)

cc_library(
    name = "uart_framing",
    hdrs = ["uart_framing.hpp"],
    deps = [
        "//proto:tbots_nanopb_proto",
        "//proto/message_translation:power_frame_msg",
        "@nanopb",
    ],
)

cc_test(
    name = "uart_framing_test",
    srcs = [
        "uart_framing_test.cpp",
    ],
    deps = [
        ":uart_framing",
        "//proto:tbots_nanopb_proto",
        "//shared/test_util:tbots_gtest_main",
    ],
)
