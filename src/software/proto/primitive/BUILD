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
        "//shared/proto:tbots_cc_proto",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:point",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/util/make_enum",
    ],
)

cc_test(
    name = "primitive_msg_factory_test",
    srcs = ["primitive_msg_factory_test.cpp"],
    deps = [
        ":primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
    ],
)
