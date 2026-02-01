package(default_visibility = ["//visibility:public"])

cc_library(
    name = "toml_config",
    srcs = ["toml_config_client.cpp"],
    hdrs = ["toml_config_client.h"],
    deps = [
        "//software/logger",
        "@tomlplusplus",
    ],
)

cc_test(
    name = "toml_config_client_test",
    srcs = ["toml_config_client_test.cpp"],
    deps = [
        ":toml_config",
        "//shared/test_util:tbots_gtest_main",
    ],
)
