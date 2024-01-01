package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_fusion",
    srcs = ["sensor_fusion.cpp"],
    hdrs = ["sensor_fusion.h"],
    deps = [
        "//proto:sensor_msg_cc_proto",
        "//proto/message_translation:ssl_detection",
        "//proto/message_translation:ssl_geometry",
        "//proto/message_translation:ssl_referee",
        "//proto/message_translation:tbots_protobuf",
        "//software/logger",
        "//software/sensor_fusion/filter:sensor_fusion_filters",
        "//software/sensor_fusion/filter:vision_detection",
        "//software/sensor_fusion/possession:possession_tracker",
        "//software/world",
        "@tracy", # TODO (NIMA): Remove
    ],
)

cc_test(
    name = "sensor_fusion_test",
    srcs = ["sensor_fusion_test.cpp"],
    deps = [
        ":sensor_fusion",
        "//proto/message_translation:ssl_detection",
        "//proto/message_translation:ssl_geometry",
        "//proto/message_translation:ssl_wrapper",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "threaded_sensor_fusion",
    srcs = ["threaded_sensor_fusion.cpp"],
    hdrs = ["threaded_sensor_fusion.h"],
    deps = [
        ":sensor_fusion",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
    ],
)
