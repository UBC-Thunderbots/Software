package(default_visibility = ["//visibility:public"])

cc_library(
    name = "grsim_output",
    srcs = ["grsim_output.cpp"],
    hdrs = ["grsim_output.h"],
    deps = [
        "//shared:constants",
        "//software/world:team",
        "//software/backend/output/grsim/command_primitive_visitor:grsim_command_primitive_visitor",
        "//software/proto:grsim_cc_proto",
        "//software/util/parameter:dynamic_parameters",
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
