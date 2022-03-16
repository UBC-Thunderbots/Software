package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_estop_reader",
    srcs = ["threaded_estop_reader.cpp"],
    hdrs = ["threaded_estop_reader.h"],
    deps = [
        "//shared:constants",
        "//software/logger",
        "//software/uart:uart_communication_interface",
        "//software/util/make_enum",
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
        "threaded_estop_reader_test.cpp",
    ],
    deps = [
        ":threaded_estop_reader",
        "//shared/test_util:tbots_gtest_main",
        "//software/uart:mock_uart_communication",
    ],
)
