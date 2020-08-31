cc_library(
    name = "parameter",
    srcs = ["parameter.c"],
    hdrs = ["parameter.h"],
)

genrule(
    name = "config_c_generate",
    outs = [
        "config.h",
        "config.c",
    ],
    cmd = "$(location //shared/parameter_v2/scripts:generate_dynamic_parameters)" +
          " --output config -c " +
          "&& mv config.h $(location config.h)" +
          "&& mv config.c $(location config.c)",
    tools = ["//shared/parameter_v2/scripts:generate_dynamic_parameters"],
)

cc_library(
    name = "config",
    srcs = ["config.c"],
    hdrs = ["config.h"],
    deps = [":parameter"],
)

cc_test(
    name = "config_test",
    srcs = [
        "config_test.cpp",
    ],
    data = [
        "config.h",
        "//shared/parameter_v2/config:test_configs",
    ],
    deps = [
        ":config",
        ":parameter",
        "//software/util/variant_visitor",
        "@boost//:filesystem",
        "@gtest//:gtest_main",
        "@libclang",
        "@yaml-cpp",
    ],
)
