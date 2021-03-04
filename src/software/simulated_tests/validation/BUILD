package(default_visibility = ["//visibility:public"])

cc_library(
    name = "validation_function",
    testonly = True,
    srcs = ["validation_function.cpp"],
    hdrs = ["validation_function.h"],
    deps = [
        "//software/world",
        "@boost//:coroutine2",
        "@gtest",
    ],
)

cc_library(
    name = "terminating_function_validator",
    testonly = True,
    srcs = ["terminating_function_validator.cpp"],
    hdrs = ["terminating_function_validator.h"],
    deps = [
        ":validation_function",
        "@boost//:coroutine2",
    ],
)

cc_test(
    name = "terminating_function_validator_test",
    srcs = ["terminating_function_validator_test.cpp"],
    deps = [
        ":terminating_function_validator",
        ":validation_function",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "non_terminating_function_validator",
    testonly = True,
    srcs = ["non_terminating_function_validator.cpp"],
    hdrs = ["non_terminating_function_validator.h"],
    deps = [
        ":validation_function",
        "@boost//:coroutine2",
    ],
)

cc_test(
    name = "non_terminating_function_validator_test",
    srcs = ["non_terminating_function_validator_test.cpp"],
    deps = [
        ":non_terminating_function_validator",
        ":validation_function",
        "//software/test_util",
        "//software/world",
        "@gtest//:gtest_main",
    ],
)
