package(default_visibility = ["//visibility:public"])

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
