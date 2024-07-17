package(default_visibility = ["//visibility:public"])

cc_library(
    name = "type_map",
    hdrs = [
        "type_map.h",
        "type_map.tpp",
    ],
    deps = [],
)

cc_test(
    name = "type_map_test",
    srcs = [":type_map_test.cpp"],
    deps = [
        ":type_map",
        "//shared/test_util:tbots_gtest_main",
    ],
)
