load(":parameter.bzl", "c_dynamic_parameters", "cpp_dynamic_parameters")

package(default_visibility = ["//visibility:private"])

cpp_dynamic_parameters(
    name = "cpp_params",
    enum_deps = [
        "//software/world:game_state",
    ],
    generated_parameter_file = "cpp_dynamic_parameters.h",
)

c_dynamic_parameters(
    name = "c_params",
    generated_parameter_file = "c_parameters.h",
)

cc_library(
    name = "c_configs",
    hdrs = [":c_params"],
)

cc_library(
    name = "cpp_configs",
    srcs = ["dynamic_parameters.cpp"],
    hdrs = [
        "dynamic_parameters.h",
        ":cpp_params",
    ],
    deps = [
        "parameter",
        ":config",
    ],
)

cc_library(
    name = "parameter",
    hdrs = ["parameter.h"],
)

cc_library(
    name = "numeric_parameter",
    hdrs = ["numeric_parameter.h"],
    deps = ["//software/logger"],
)

cc_library(
    name = "enumerated_parameter",
    hdrs = ["enumerated_parameter.h"],
    deps = ["//software/logger"],
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

cc_test(
    name = "c_config_test",
    srcs = [
        "config_test.cpp",
    ],
    data = [
        "c_parameters.h",
        "//shared/parameter_v2/config_definitions:test_configs",
    ],
    deps = [
        "//shared/parameter_v2:c_configs",
        "@boost//:filesystem",
        "@gtest//:gtest_main",
        "@libclang",
        "@yaml-cpp",
    ],
)

cc_test(
    name = "cpp_config_test",
    srcs = [
        "cpp_config_test.cpp",
    ],
    deps = [
        "//shared/parameter_v2:cpp_configs",
        "@gtest//:gtest_main",
    ],
)
