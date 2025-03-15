package(default_visibility = ["//visibility:public"])

cc_library(
    name = "redis",
    srcs = ["redis_client.cpp"],
    hdrs = ["redis_client.h"],
    deps = [
        "//software/logger",
        "@cpp_redis",
    ],
)

cc_test(
    name = "redis_test_cpp",
    srcs = [
        "redis_client_test.cpp",
    ],
    deps = [
        ":redis",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "@cpp_redis",
    ],
)

cc_library(
    name = "yaml",
    srcs = [
        "yaml_reader.cpp",
    ],
    hdrs = [
        "yaml_reader.h",
    ],
    deps = [
        "@yaml-cpp//:yaml-cpp",
        "//software/logger",
    ],
)

cc_test(
    name = "yaml_reader_test",
    srcs = [
        "yaml_reader_test.cpp",
    ],
    deps = [
        ":yaml_reader",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
