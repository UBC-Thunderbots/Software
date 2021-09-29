package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_logger",
    hdrs = [
        "proto_logger.h",
        "proto_logger.tpp",
    ],
    deps = [
        "//proto:repeated_any_msg_cc_proto",
        "//software/logger",
        "//software/multithreading:threaded_observer",
        "//software/util/typename",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "proto_log_reader",
    srcs = [
        "proto_log_reader.cpp",
    ],
    hdrs = [
        "proto_log_reader.h",
    ],
    deps = [
        "//proto:repeated_any_msg_cc_proto",
        "//software/logger",
        "//software/time:timestamp",
        "//software/util/typename",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "proto_logger_log_reader_test",
    srcs = ["proto_logger_log_reader_test.cpp"],
    data = [":test_logs"],
    deps = [
        ":proto_log_reader",
        ":proto_logger",
        "//proto:repeated_any_msg_cc_proto",
        "//proto:sensor_msg_cc_proto",
        "//shared/test_util:tbots_gtest_main",
        "//software/multithreading:subject",
    ],
)
