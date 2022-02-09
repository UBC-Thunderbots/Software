package(default_visibility = ["//visibility:public"])

# https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

platformio_project(
    name = "main",
    src = "emergency_stop_main.cc",
    board = "uno",
    framework = "arduino",
    deps = [
        "//shared:constants_platformio",
    ],
)

cc_library(
    name = "threaded_estop_reader",
    srcs = ["threaded_estop_reader.cpp"],
    hdrs = ["threaded_estop_reader.h"],
    deps = [
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
    name = "estop_reader_test",
    srcs = [
        "mock_uart_communication.h",
        "threaded_estop_reader_test.cpp",
    ],
    deps = [
        ":threaded_estop_reader",
        "//shared/test_util:tbots_gtest_main",
    ],
)

