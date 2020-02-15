package(default_visibility = ["//visibility:public"])

cc_library(
    name = "generic_factory",
    srcs = ["generic_factory.cpp"],
    hdrs = [
        "generic_factory.h",
    ],
)

cc_test(
    name = "generic_factory_test",
    srcs = ["generic_factory_test.cpp"],
    deps = [
        ":generic_factory",
        "@gtest//:gtest_main",
    ],
)
