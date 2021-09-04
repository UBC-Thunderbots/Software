package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    hdrs = [
        "custom_logging_levels.h",
        "logger.h",
    ],
    deps = [
        ":coloured_cout_sink",
        ":csv_sink",
        "@g3log",
        "@g3sinks",
    ],
)

cc_library(
    name = "network_logger",
    hdrs = [
        "network_logger.h",
    ],
    deps = [
        ":network_sinc",
        "@g3log",
        "@g3sinks",
    ],
)

cc_library(
    name = "coloured_cout_sink",
    srcs = [
        "coloured_cout_sink.cpp",
    ],
    hdrs = [
        "coloured_cout_sink.h",
    ],
    deps = [
        "//software/util/make_enum",
        "@g3log",
    ],
)

cc_test(
    name = "csv_sink_test",
    srcs = ["csv_sink_test.cpp"],
    deps = [
        ":csv_sink",
        ":logger",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "csv_sink",
    srcs = [
        "csv_sink.cpp",
    ],
    hdrs = [
        "csv_sink.h",
        "custom_logging_levels.h",
    ],
    deps = [
        "@g3log",
    ],
)

cc_library(
    name = "network_sinc",
    srcs = [
        "network_sink.cpp",
    ],
    hdrs = [
        "network_sink.h",
    ],
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software:constants",
        "//software/networking:threaded_proto_udp_sender",
        "@g3log",
    ],
)

cc_test(
    name = "network_sinc_test",
    srcs = ["network_sink_test.cpp"],
    deps = [
        ":network_logger",
        ":network_sinc",
        "//shared/test_util:tbots_gtest_main",
    ],
)
