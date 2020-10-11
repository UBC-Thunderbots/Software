package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_logger",
    hdrs = [
        "proto_logger.h",
        "proto_logger.tpp",
    ],
    deps = [
        "//software/logger",
        "//software/multithreading:threaded_observer",
        "//software/proto:repeated_any_msg_cc_proto",
        "//software/time:timestamp",
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
        "//software/logger",
        "//software/proto:repeated_any_msg_cc_proto",
        "//software/time:timestamp",
        "//software/util/typename",
        "@com_google_protobuf//:protobuf",
    ],
)
