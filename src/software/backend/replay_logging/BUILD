package(default_visibility = ["//visibility:public"])

cc_library(
    name = "replay_logger",
    srcs = ["replay_logger.cpp"],
    hdrs = [
        "replay_logger.h",
    ],
    deps = [
        "//software/multithreading:threaded_observer",
        "//software/proto:replay_msg_cc_proto",
        "//software/proto:sensor_msg_cc_proto",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_library(
    name = "replay_reader",
    srcs = ["replay_reader.cpp"],
    hdrs = [
        "replay_reader.h",
    ],
    deps = [
        "//software/proto:replay_msg_cc_proto",
        "//software/proto:sensor_msg_cc_proto",
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_test(
    name = "replay_test",
    srcs = ["replay_test.cpp"],
    data = [":test_replay"],
    deps = [
        "replay_logger",
        "replay_reader",
        "//software/multithreading:subject",
        "//software/proto:replay_msg_cc_proto",
        "//software/proto:sensor_msg_cc_proto",
        "@gtest//:gtest_main",
    ],
)
