package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_util",
    testonly = True,
    srcs = [
        "test_util.cpp",
    ],
    hdrs = [
        "test_util.h",
    ],
    deps = [
        "//firmware/app/logger",
        "//shared/proto:tbots_cc_proto",
        "//shared/proto:tbots_nanopb_proto",
        "//software/logger",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "tbots_gtest_main",
    testonly = True,
    srcs = ["tbots_gtest_main.cpp"],
    hdrs = ["tbots_gtest_main.h"],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/logger",
        "@gtest",
    ],
)
