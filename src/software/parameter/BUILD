package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parameter",
    hdrs = ["parameter.h"],
    deps = [],
)

cc_library(
    name = "numeric_parameter",
    hdrs = ["numeric_parameter.h"],
    deps = ["//software/logger"],
)

cc_library(
    name = "enumerated_parameter",
    hdrs = ["enumerated_parameter.h"],
    deps = [
        ":parameter",
        "//software/logger",
    ],
)

cc_library(
    name = "config",
    hdrs = ["config.h"],
    deps = [
        ":enumerated_parameter",
        ":numeric_parameter",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "dynamic_parameters",
    srcs = ["dynamic_parameters.cpp"],
    hdrs = [
        "dynamic_parameters.h",
    ],
    deps = [
        ":config",
        ":parameter",
        "//shared/parameter_v2:cpp_configs",
    ],
)

cc_test(
    name = "parameter_test",
    srcs = ["parameter_test.cpp"],
    deps = [
        ":enumerated_parameter",
        ":numeric_parameter",
        ":parameter",
        "//software/util/make_enum",
        "@gtest//:gtest_main",
    ],
)
