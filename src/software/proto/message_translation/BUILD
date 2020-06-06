package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ssl_message_translator",
    srcs = ["ssl_message_translator.cpp"],
    hdrs = ["ssl_message_translator.h"],
    deps = [
        "//shared:constants",
        "//software/proto:ssl_cc_proto",
        "//software/util/make_enum",
        "//software/world:field",
    ],
)

cc_test(
    name = "ssl_message_translator_test",
    srcs = ["ssl_message_translator_test.cpp"],
    deps = [
        ":ssl_message_translator",
        "//software/test_util",
        "@gtest//:gtest_main",
    ],
)
