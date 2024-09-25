package(default_visibility = ["//visibility:public"])

cc_library(
    name = "is_template_base_of",
    hdrs = ["is_template_base_of.hpp"],
)

cc_test(
    name = "is_template_base_of_test",
    srcs = [":is_template_base_of_test.cpp"],
    deps = [
        ":is_template_base_of",
        "//shared/test_util:tbots_gtest_main",
    ],
)
