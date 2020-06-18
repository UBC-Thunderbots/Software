package(default_visibility = ["//visibility:public"])

cc_library(
    name = "validation_function",
    hdrs = ["validation_function.h"],
    deps = [
        "//software/world",
        "@boost//:coroutine2",
    ],
)

cc_library(
    name = "function_validator",
    srcs = ["function_validator.cpp"],
    hdrs = ["function_validator.h"],
    deps = [
        ":validation_function",
        "@boost//:coroutine2",
    ],
)

cc_test(
    name = "function_validator_test",
    srcs = ["function_validator_test.cpp"],
    deps = [
        ":function_validator",
        ":validation_function",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "continuous_function_validator",
    srcs = ["continuous_function_validator.cpp"],
    hdrs = ["continuous_function_validator.h"],
    deps = [
        ":validation_function",
        "@boost//:coroutine2",
    ],
)

cc_test(
    name = "continuous_function_validator_test",
    srcs = ["continuous_function_validator_test.cpp"],
    deps = [
        ":continuous_function_validator",
        ":validation_function",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
