package(default_visibility = ["//visibility:public"])

cc_library(
    name = "detection",
    srcs = [
        "vision_detection.cpp",
    ],
    hdrs = [
        "ball_detection.h",
        "robot_detection.h",
        "vision_detection.h",
    ],
    deps = [
        "//software/geom",
        "//software/util/time:timestamp",
    ],
)

cc_library(
    name = "refbox_data",
    srcs = ["refbox_data.cpp"],
    hdrs = ["refbox_data.h"],
    deps = [
        "//software/geom",
        "//software/util/time:timestamp",
        "//software/proto:ssl_cc_proto",
        "@magic_enum",
    ],
)

cc_test(
    name = "refbox_data_test",
    srcs = ["refbox_data_test.cpp"],
    deps = [
        ":refbox_data",
        "@gtest//:gtest_main",
    ],
)

