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
        ":protobuf_sink",
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
        ":coloured_cout_sink",
        ":network_sink",
        "@g3log",
        "@g3sinks",
    ],
)

cc_test(
    name = "coloured_cout_sink_test",
    srcs = ["coloured_cout_sink_test.cpp"],
    deps = [
        ":coloured_cout_sink",
        ":logger",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "coloured_cout_sink",
    srcs = [
        "coloured_cout_sink.cpp",
    ],
    hdrs = [
        "coloured_cout_sink.h",
        "custom_logging_levels.h",
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
    name = "network_sink",
    srcs = [
        "network_sink.cpp",
    ],
    hdrs = [
        "network_sink.h",
        "custom_logging_levels.h",
        "constants.h",
    ],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto:visualization_cc_proto",
        "//shared:constants",
        "//software/networking:threaded_proto_udp_sender",
        "@g3log",
        ":logger",
    ],
)

cc_library(
    name = "protobuf_sink",
    srcs = [
        "protobuf_sink.cpp",
    ],
    hdrs = [
        "custom_logging_levels.h",
        "protobuf_sink.h",
        "constants.h",
    ],
    deps = [
        "//proto:any_cc_proto",
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//software/networking:threaded_unix_sender",
        "@base64",
        "@g3log",
    ],
)

py_library(
    name = "py_logger",
    srcs = [
        "logger.py",
    ],
)
