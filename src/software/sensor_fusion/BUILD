package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vision_detection",
    srcs = ["vision_detection.cpp"],
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

cc_test(
    name = "vision_detection_test",
    srcs = ["vision_detection_test.cpp"],
    deps = [
        ":vision_detection",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "refbox_data",
    hdrs = ["refbox_data.h"],
    deps = [
        ":team_info",
        "//software/geom",
        "//software/proto:ssl_cc_proto",
        "//software/time:timestamp",
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "team_info",
    srcs = ["team_info.cpp"],
    hdrs = ["team_info.h"],
    deps = [
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
    name = "ssl_protobuf_reader",
    srcs = ["ssl_protobuf_reader.cpp"],
    hdrs = ["ssl_protobuf_reader.h"],
    deps = [
        "//software/logger",
        "//software/parameter:dynamic_parameters",
        "//software/proto:ssl_cc_proto",
        "//software/proto/message_translation:ssl_geometry_message_translator",
        "//software/sensor_fusion:refbox_data",
        "//software/sensor_fusion:vision_detection",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:team",
    ],
)

cc_library(
    name = "sensor_fusion",
    srcs = ["sensor_fusion.cpp"],
    hdrs = ["sensor_fusion.h"],
    deps = [
        ":refbox_data",
        ":ssl_protobuf_reader",
        ":vision_detection",
        "//software/backend:robot_status",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/parameter:dynamic_parameters",
        "//software/primitive",
        "//software/proto:sensor_msg_cc_proto",
        "//software/sensor_fusion/filter:sensor_fusion_filters",
        "//software/world",
    ],
)
