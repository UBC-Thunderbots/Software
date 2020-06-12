package(default_visibility = ["//visibility:public"])

cc_library(
    name = "duration",
    srcs = ["duration.cpp"],
    hdrs = ["duration.h"],
    deps = [
        ":time",
        "//shared:constants",
    ],
)

cc_library(
    name = "time",
    srcs = ["time.cpp"],
    hdrs = ["time.h"],
    deps = [
        "//shared:constants",
    ],
)

cc_library(
    name = "timestamp",
    srcs = ["timestamp.cpp"],
    hdrs = ["timestamp.h"],
    deps = [
        ":duration",
        ":time",
        "//shared:constants",
    ],
)

cc_test(
    name = "duration_test",
    srcs = ["duration_test.cpp"],
    deps = [
        ":duration",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "timestamp_test",
    srcs = ["timestamp_test.cpp"],
    deps = [
        ":timestamp",
        "@gtest//:gtest_main",
    ],
)
