package(default_visibility = ["//visibility:public"])

load("@rules_python//python:defs.bzl", "py_binary")
load("@dynamic_parameter_deps//:requirements.bzl", "requirement")

cc_library(
    name = "threaded_estop_reader",
    srcs = ["threaded_estop_reader.cpp"],
    hdrs = ["threaded_estop_reader.h"],
    deps = [
        "arduino_util",
        "uart_communication_interface",
        "//shared:constants",
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

cc_library(
    name = "arduino_util",
    srcs = ["arduino_util.cpp"],
    hdrs = ["arduino_util.h"],
    deps = [
        "//shared:constants",
        "@boost//:filesystem",
        "@boost//:format",
    ],
)

cc_test(
    name = "find_port_test",
    srcs = [
        "arduino_util_test.cpp",
    ],
    deps = [
        ":arduino_util",
        "@gtest//:gtest_main",
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
