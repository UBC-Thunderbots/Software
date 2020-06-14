package(default_visibility = ["//visibility:public"])

cc_library(
    name = "protobuf_primitive_visitor",
    srcs = ["protobuf_primitive_visitor.cpp"],
    hdrs = ["protobuf_primitive_visitor.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared:constants",
        "//shared:firmware_primitive_type",
        "//shared/proto:tbots_cc_proto",
        "//software/primitive",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
    ],
)

cc_library(
    name = "protobuf_message_translation",
    srcs = ["protobuf_message_translation.cpp"],
    hdrs = ["protobuf_message_translation.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software/new_geom:angle",
        "//software/new_geom:point",
        "//software/new_geom:vector",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
        "//software/proto/message_translation:protobuf_primitive_visitor",
        "//software/world",
    ],
)

cc_test(
    name = "protobuf_message_translation_test",
    srcs = ["protobuf_message_translation_test.cpp"],
    deps = [
        "//software/proto/message_translation:protobuf_message_translation",
        "//software/test_util",
    ],
)

cc_library(
    name = "ssl_geometry_message_translator",
    srcs = ["ssl_geometry_message_translator.cpp"],
    hdrs = ["ssl_geometry_message_translator.h"],
    deps = [
        "//shared:constants",
        "//software/proto:ssl_cc_proto",
        "//software/util/make_enum",
        "//software/world:field",
    ],
)

cc_test(
    name = "ssl_geometry_message_translator_test",
    srcs = ["ssl_geometry_message_translator_test.cpp"],
    deps = [
        ":ssl_geometry_message_translator",
        "//software/sensor_fusion:ssl_protobuf_reader",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ssl_detection_message_translator",
    srcs = ["ssl_detection_message_translator.cpp"],
    hdrs = ["ssl_detection_message_translator.h"],
    deps = [
        "//shared:constants",
        "//software/proto:ssl_cc_proto",
        "//software/time:timestamp",
        "//software/world:ball_state",
        "//software/world:robot_state",
    ],
)

cc_test(
    name = "ssl_detection_message_translator_test",
    srcs = ["ssl_detection_message_translator_test.cpp"],
    deps = [
        ":ssl_detection_message_translator",
        "//software/sensor_fusion:ssl_protobuf_reader",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ssl_wrapper_message_translator",
    srcs = ["ssl_wrapper_message_translator.cpp"],
    hdrs = ["ssl_wrapper_message_translator.h"],
    deps = [
        ":ssl_detection_message_translator",
        ":ssl_geometry_message_translator",
        "//software/proto:ssl_cc_proto",
    ],
)

cc_test(
    name = "ssl_wrapper_message_translator_test",
    srcs = ["ssl_wrapper_message_translator_test.cpp"],
    deps = [
        ":ssl_wrapper_message_translator",
        "@gtest//:gtest_main",
    ],
)
