package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_estop_reader",
    srcs = ["threaded_estop_reader.cpp"],
    hdrs = ["threaded_estop_reader.h"],
    deps = [
        "uart_communication_interface",
        "//software/logger",
        "//software/util/make_enum",
        "@boost//:asio",
    ],
)

cc_library(
    name = "uart_communication_interface",
    hdrs = ["uart_communication.h"],
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

cc_test(
    name = "estop_reader_test",
    srcs = [
        "mock_uart_communication.h",
        "threaded_estop_reader_test.cpp",
    ],
    deps = [
        ":threaded_estop_reader",
        "@gtest//:gtest_main",
    ],
)
