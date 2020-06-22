package(default_visibility = ["//visibility:public"])

cc_library(
    name = "grsim_output",
    srcs = ["grsim_output.cpp"],
    hdrs = ["grsim_output.h"],
    deps = [
        "//shared:constants",
        "//software/backend/output/grsim/command_primitive_visitor:grsim_command_primitive_visitor",
        "//software/parameter:dynamic_parameters",
        "//software/proto:grsim_cc_proto",
        "//software/util/variant_visitor",
        "//software/world:team",
        "@boost//:asio",
    ],
)

cc_test(
    name = "grsim_output_test",
    srcs = ["grsim_output_test.cpp"],
    deps = [
        ":grsim_output",
        "@gtest//:gtest_main",
    ],
)
