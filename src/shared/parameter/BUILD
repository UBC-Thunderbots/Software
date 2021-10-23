load(":parameter.bzl", "cpp_dynamic_parameters")

package(default_visibility = ["//visibility:public"])

cpp_dynamic_parameters(
    name = "cpp_params",
    enum_deps = [
        "//software/world:game_state",
        "//software/world:team_colour",
    ],
    generated_parameter_header = "cpp_dynamic_parameters.h",
    generated_parameter_source = "cpp_dynamic_parameters.cpp",
)

cc_library(
    name = "cpp_configs",
    srcs = [
        ":cpp_params",
    ],
    hdrs = [
        ":cpp_params",
    ],
    deps = [
        ":config",
        ":enumerated_parameter",
        ":numeric_parameter",
        ":parameter",
        "//software/util/generic_factory",
        "//software/world:game_state",
        "//software/world:team_colour",
    ],
)

cc_test(
    name = "cpp_config_test",
    srcs = [
        "cpp_config_test.cpp",
    ],
    data = [
        "//shared/parameter/config_definitions:parameter_configs",
    ],
    deps = [
        "//shared/parameter:cpp_configs",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:halt_play",
        "//software/util/variant_visitor",
        "@boost//:filesystem",
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

cc_test(
    name = "parameter_test",
    srcs = ["parameter_test.cpp"],
    deps = [
        ":enumerated_parameter",
        ":numeric_parameter",
        ":parameter",
        "//shared/test_util:tbots_gtest_main",
        "//software/util/make_enum",
    ],
)
