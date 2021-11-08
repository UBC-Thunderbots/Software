package(default_visibility = ["//visibility:public"])

cc_library(
    name = "redis",
    srcs = ["cpp_redis.cpp"],
    deps = [
        "@redis-plus-plus",
        "@hiredis_library",
    ]
)

cc_test(
    name = "redis_test_cpp",
    srcs = ["redis_test.cpp"],
    deps = [
        "@redis-plus-plus",
        "@hiredis_library",
        "//shared/test_util:tbots_gtest_main"
    ],
)