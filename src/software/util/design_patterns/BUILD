package(default_visibility = ["//visibility:public"])

cc_library(
    name = "generic_factory",
    hdrs = [
        "generic_factory.h",
        "generic_factory.tpp",
    ],
    deps = [
        "//software/util/typename",
    ],
)

cc_library(
    name = "constructor_args_generic_factory",
    hdrs = [
        "constructor_args_generic_factory.h",
        "constructor_args_generic_factory.tpp",
    ],
    deps = [
        "//software/util/typename",
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

cc_test(
    name = "constructor_args_generic_factory_test",
    srcs = ["constructor_args_generic_factory_test.cpp"],
    deps = [
        ":constructor_args_generic_factory",
        "@gtest//:gtest_main",
    ],
)
