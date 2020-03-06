package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vision_detection",
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
        "//software/time:timestamp",
        "//software/world",
    ],
)

cc_library(
    name = "refbox_data",
    hdrs = ["refbox_data.h"],
    deps = [
        "//software/geom",
        "//software/proto:ssl_cc_proto",
        "//software/time:timestamp",
        "//software/util/printable_enum_macro",
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

cc_library(
    name = "sensor_fusion",
    srcs = ["sensor_fusion.cpp"],
    hdrs = ["sensor_fusion.h"],
    deps = [
        ":refbox_data",
        ":vision_detection",
        "//software/ai/primitive",
        "//software/backend:robot_status",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/parameter:dynamic_parameters",
        "//software/sensor_fusion/filter:sensor_fusion_filters",
        "//software/world",
    ],
)
