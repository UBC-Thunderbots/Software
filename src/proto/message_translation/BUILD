package(default_visibility = ["//visibility:public"])

load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)

cc_library(
    name = "tbots_protobuf",
    srcs = ["tbots_protobuf.cpp"],
    hdrs = ["tbots_protobuf.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":tbots_geometry",
        "//proto:tbots_cc_proto",
        "//proto:visualization_cc_proto",
        "//software/ai/passing:pass_with_rating",
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
        "//proto:tbots_cc_proto",
        "//software/ai/navigator/path_planner/hrvo:velocity_obstacle",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:circle",
        "//software/geom:point",
        "//software/geom:polygon",
        "//software/geom:vector",
        "//software/geom:stadium",
    ],
)

cc_test(
    name = "tbots_geometry_test",
    srcs = ["tbots_geometry_test.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "tbots_geometry",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "ssl_geometry",
    srcs = ["ssl_geometry.cpp"],
    hdrs = ["ssl_geometry.h"],
    deps = [
        "//proto:ssl_cc_proto",
        "//shared:constants",
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
        "//proto:ssl_cc_proto",
        "//shared:constants",
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
        "//proto:ssl_cc_proto",
        "//software/logger",
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
        "//proto:ssl_cc_proto",
        "//software/geom:point",
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
        "//proto:ssl_simulation_cc_proto",
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//shared:robot_constants",
        "//software/geom:angle",
        "//software/logger",
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
    name = "er_force_world",
    srcs = ["er_force_world.cpp"],
    hdrs = ["er_force_world.h"],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:protobuf_cc_lib",
        "//software/world:ball",
        "//software/world:robot",
    ],
)

cc_test(
    name = "er_force_world_test",
    srcs = ["er_force_world_test.cpp"],
    deps = [
        ":er_force_world",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

platformio_library(
    name = "power_frame_msg_platformio",
    hdr = "power_frame_msg.hpp",
)

cc_library(
    name = "power_frame_msg",
    hdrs = ["power_frame_msg.hpp"],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto:tbots_nanopb_proto",
        "//proto/primitive:primitive_msg_factory",
        "@nanopb",
    ],
)

py_library(
    name = "py_tbots_protobuf",
    srcs = [
        "tbots_protobuf.py",
    ],
    deps = [
        "//proto:import_all_protos",
        "//proto:software_py_proto",
        "//proto:tbots_py_proto",
    ],
)

cc_test(
    name = "power_frame_msg_test",
    srcs = ["power_frame_msg_test.cpp"],
    deps = [
        ":power_frame_msg",
        "//shared/test_util:tbots_gtest_main",
    ],
)
