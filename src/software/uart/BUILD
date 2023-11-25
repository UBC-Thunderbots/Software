package(default_visibility = ["//visibility:public"])

cc_library(
    name = "uart_communication_interface",
    hdrs = ["uart_communication.h"],
)

cc_library(
    name = "mock_uart_communication",
    hdrs = ["mock_uart_communication.h"],
)

cc_library(
    name = "boost_uart_communication",
    srcs = ["boost_uart_communication.cpp"],
    hdrs = ["boost_uart_communication.h"],
    deps = [
        "uart_communication_interface",
        "@boost//:asio",
    ],
)
