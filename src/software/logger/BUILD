package(default_visibility = ["//visibility:public"])

cc_library(
    name = "log_merger",
    srcs = [
        "log_merger.cpp",
    ],
    hdrs = [
        "log_merger.h",
    ],
    deps = [
        ":compat_flags",
        "@g3log",
    ],
)

cc_library(
    name = "logger",
    hdrs = [
        "custom_logging_levels.h",
        "logger.h",
    ],
    linkopts = select({
        "@platforms//os:linux": ["-lstdc++fs"],
        "//conditions:default": [],
    }),
    deps = [
        ":coloured_cout_sink",
        ":compat_flags",
        ":csv_sink",
        ":log_merger",
        ":plotjuggler_sink",
        ":protobuf_sink",
        "@g3log",
        "@g3sinks",
    ],
)

cc_library(
    name = "network_logger",
    srcs = [
        "network_logger.cpp",
    ],
    hdrs = [
        "network_logger.h",
    ],
    deps = [
        ":coloured_cout_sink",
        ":network_sink",
        "//software/world:robot_state",
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

cc_test(
    name = "log_merger_test",
    srcs = ["log_merger_test.cpp"],
    deps = [
        ":log_merger",
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
        ":log_merger",
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
        ":compat_flags",
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
    ],
    deps = [
        ":log_merger",
        "//proto:tbots_cc_proto",
        "//proto:visualization_cc_proto",
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software/logger:plotjuggler_sink",
        "//software/networking/udp:threaded_proto_udp_sender",
        "@base64",
        "@g3log",
    ],
)

cc_library(
    name = "plotjuggler_sink",
    srcs = [
        "plotjuggler_sink.cpp",
    ],
    hdrs = [
        "custom_logging_levels.h",
        "plotjuggler_sink.h",
    ],
    deps = [
        "//proto:visualization_cc_proto",
        "//shared:constants",
        "//software/networking/udp:threaded_udp_sender",
        "@g3log",
        "@protobuf//:json_util",
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
    ],
    deps = [
        "//proto:any_cc_proto",
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//software/logger:proto_logger",
        "//software/networking/unix:threaded_unix_sender",
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

cc_library(
    name = "proto_logger",
    srcs = [
        "proto_logger.cpp",
    ],
    hdrs = [
        "proto_logger.h",
    ],
    deps = [
        "//proto:tbots_cc_proto",
        "//software/multithreading:thread_safe_buffer",
        "@base64",
        ":generic_logger"
    ],
)

cc_library(
    name = "compat_flags",
    srcs = ["compat_flags.h"],
)