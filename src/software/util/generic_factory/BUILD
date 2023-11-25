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

cc_test(
    name = "generic_factory_test",
    srcs = ["generic_factory_test.cpp"],
    deps = [
        ":generic_factory",
        "//shared/test_util:tbots_gtest_main",
    ],
)
