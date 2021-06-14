package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tbots_protobuf",
    srcs = ["tbots_protobuf.cpp"],
    hdrs = ["tbots_protobuf.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":tbots_geometry",
        "//shared/proto:tbots_cc_proto",
        "//software/geom:angle",
        "//software/geom:point",
        "//software/geom:vector",
        "//software/world",
    ],
)

cc_test(
    name = "tbots_protobuf_test",
    srcs = ["tbots_protobuf_test.cpp"],
    deps = [
        "tbots_protobuf",
        "//software/test_util",
    ],
)

cc_library(
    name = "tbots_geometry",
    srcs = ["tbots_geometry.cpp"],
    hdrs = ["tbots_geometry.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/geom:vector",
    ],
)

cc_test(
    name = "tbots_geometry_test",
    srcs = ["tbots_geometry_test.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "tbots_geometry",
        "//shared/test_util:tbots_gtest_main",
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
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
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
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "ssl_wrapper",
    srcs = ["ssl_wrapper.cpp"],
    hdrs = ["ssl_wrapper.h"],
    deps = [
        ":ssl_detection",
        ":ssl_geometry",
        "//software/logger",
        "//software/proto:ssl_cc_proto",
        "//software/world",
    ],
)

cc_test(
    name = "ssl_wrapper_test",
    srcs = ["ssl_wrapper_test.cpp"],
    deps = [
        ":ssl_wrapper",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "ssl_referee",
    srcs = ["ssl_referee.cpp"],
    hdrs = ["ssl_referee.h"],
    deps = [
        "//software/geom:point",
        "//software/proto:ssl_cc_proto",
        "//software/world:game_state",
        "//software/world:team_colour",
    ],
)

cc_test(
    name = "ssl_referee_test",
    srcs = ["ssl_referee_test.cpp"],
    deps = [
        ":ssl_referee",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "ssl_simulation_robot_control",
    srcs = ["ssl_simulation_robot_control.cpp"],
    hdrs = ["ssl_simulation_robot_control.h"],
    deps = [
        "//firmware/shared:physics",
        "//software/proto:ssl_simulation_cc_proto",
    ],
)

cc_test(
    name = "ssl_simulation_robot_control_test",
    srcs = ["ssl_simulation_robot_control_test.cpp"],
    deps = [
        ":ssl_simulation_robot_control",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "primitive_google_to_nanopb_converter",
    srcs = ["primitive_google_to_nanopb_converter.cpp"],
    hdrs = ["primitive_google_to_nanopb_converter.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//shared:constants",
        "//shared/proto:tbots_cc_proto",
        "//shared/proto:tbots_nanopb_proto",
        "//software/proto/primitive:primitive_msg_factory",
        "@nanopb",
    ],
)

cc_test(
    name = "primitive_google_to_nanopb_converter_test",
    srcs = ["primitive_google_to_nanopb_converter_test.cpp"],
    deps = [
        ":primitive_google_to_nanopb_converter",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "defending_side",
    srcs = ["defending_side.cpp"],
    hdrs = ["defending_side.h"],
    deps = [
        "//software/proto:defending_side_msg_cc_proto",
        "//software/world:field",
    ],
)

cc_test(
    name = "defending_side_test",
    srcs = ["defending_side_test.cpp"],
    deps = [
        ":defending_side",
        "//shared/test_util:tbots_gtest_main",
    ],
)
