load(":parameter.bzl", "c_dynamic_parameters", "cpp_dynamic_parameters")

package(default_visibility = ["//visibility:public"])

cpp_dynamic_parameters(
    name = "cpp_params",
    enum_deps = [
        "//software/world:game_state",
        "//software/ai/hl/stp/play",
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
    hdrs = [
        ":cpp_params",
    ],
    deps = [
        "//software/ai/hl/stp/play",
        "//software/util/design_patterns:generic_factory",
        "//software/world:game_state",
    ],
)

cc_test(
    name = "c_config_test",
    srcs = [
        "config_test.cpp",
    ],
    data = [
        "c_parameters.h",
        "//shared/parameter_v2/config_definitions:parameter_configs",
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
    data = [
        "//shared/parameter_v2/config_definitions:parameter_configs",
    ],
    deps = [
        "//shared/parameter_v2:cpp_configs",
        "//software/ai/hl/stp/play:halt_play",
        "//software/util/variant_visitor",
        "@boost//:filesystem",
        "@gtest//:gtest_main",
        "@yaml-cpp",
    ],
)

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
