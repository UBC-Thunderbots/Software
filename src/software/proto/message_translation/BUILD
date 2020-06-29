package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tbots_protobuf",
    srcs = ["tbots_protobuf.cpp"],
    hdrs = ["tbots_protobuf.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":proto_creator_primitive_visitor",
        "//shared/proto:tbots_cc_proto",
        "//software/new_geom:angle",
        "//software/new_geom:point",
        "//software/new_geom:vector",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
        "//software/world",
    ],
)

cc_test(
    name = "tbots_protobuf_test",
    srcs = ["tbots_protobuf_test.cpp"],
    deps = [
        "//software/proto/message_translation:tbots_protobuf",
        "//software/test_util",
    ],
)

cc_library(
    name = "ssl_geometry",
    srcs = ["ssl_geometry.cpp"],
    hdrs = ["ssl_geometry.h"],
    deps = [
        "//shared:constants",
        "//software/proto:ssl_cc_proto",
        "//software/util/make_enum",
        "//software/world:field",
    ],
)

cc_test(
    name = "ssl_geometry_test",
    srcs = ["ssl_geometry_test.cpp"],
    deps = [
        ":ssl_geometry",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ssl_detection",
    srcs = ["ssl_detection.cpp"],
    hdrs = ["ssl_detection.h"],
    deps = [
        "//shared:constants",
        "//software/proto:ssl_cc_proto",
        "//software/sensor_fusion/filter:vision_detection",
        "//software/time:timestamp",
        "//software/world:ball_state",
        "//software/world:robot_state",
    ],
)

cc_test(
    name = "ssl_detection_test",
    srcs = ["ssl_detection_test.cpp"],
    deps = [
        ":ssl_detection",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ssl_wrapper",
    srcs = ["ssl_wrapper.cpp"],
    hdrs = ["ssl_wrapper.h"],
    deps = [
        ":ssl_detection",
        ":ssl_geometry",
        "//software/proto:ssl_cc_proto",
    ],
)

cc_test(
    name = "ssl_wrapper_test",
    srcs = ["ssl_wrapper_test.cpp"],
    deps = [
        ":ssl_wrapper",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "ssl_referee",
    srcs = ["ssl_referee.cpp"],
    hdrs = ["ssl_referee.h"],
    deps = [
        "//software/new_geom:point",
        "//software/proto:ssl_cc_proto",
        "//software/sensor_fusion:refbox_data",
        "//software/world:team_colour",
    ],
)

cc_test(
    name = "ssl_referee_test",
    srcs = ["ssl_referee_test.cpp"],
    deps = [
        ":ssl_referee",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "proto_creator_primitive_visitor",
    srcs = ["proto_creator_primitive_visitor.cpp"],
    hdrs = ["proto_creator_primitive_visitor.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared:constants",
        "//shared/proto:tbots_cc_proto",
        "//software/primitive:all_primitives",
        "//software/primitive:primitive_visitor",
    ],
)

cc_test(
    name = "proto_creator_primitive_visitor_test",
    srcs = ["proto_creator_primitive_visitor_test.cpp"],
    deps = [
        ":proto_creator_primitive_visitor",
        "@gtest//:gtest_main",
    ],
)
