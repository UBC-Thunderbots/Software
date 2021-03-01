package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_fusion",
    srcs = ["sensor_fusion.cpp"],
    hdrs = ["sensor_fusion.h"],
    deps = [
        "//shared/parameter_v2:cpp_configs",
        "//software/logger",
        "//software/proto:sensor_msg_cc_proto",
        "//software/proto/message_translation:ssl_detection",
        "//software/proto/message_translation:ssl_geometry",
        "//software/proto/message_translation:ssl_referee",
        "//software/sensor_fusion/filter:sensor_fusion_filters",
        "//software/sensor_fusion/filter:vision_detection",
        "//software/world",
    ],
)

cc_test(
    name = "sensor_fusion_test",
    srcs = ["sensor_fusion_test.cpp"],
    deps = [
        ":sensor_fusion",
        "//software/proto/message_translation:ssl_detection",
        "//software/proto/message_translation:ssl_geometry",
        "//software/proto/message_translation:ssl_wrapper",
        "@gtest//:gtest_main",
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
