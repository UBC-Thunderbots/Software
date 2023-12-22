load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "primitive_msg_factory",
    srcs = ["primitive_msg_factory.cpp"],
    hdrs = [
        "primitive_msg_factory.h",
        "primitive_types.h",
    ],
    deps = [
        "//proto:tbots_cc_proto",
        "//proto/message_translation:tbots_protobuf",
        "//shared:robot_constants",
        "//software/ai/navigator/trajectory:bang_bang_trajectory_1d_angular",
        "//software/ai/navigator/trajectory:bang_bang_trajectory_2d",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/util/make_enum",
        "//software/world:robot",
    ],
)

cc_test(
    name = "primitive_msg_factory_test",
    srcs = ["primitive_msg_factory_test.cpp"],
    deps = [
        ":primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
