package(default_visibility = ["//visibility:public"])

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
