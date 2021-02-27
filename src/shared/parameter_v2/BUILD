load(":parameter.bzl", "c_dynamic_parameters", "cpp_dynamic_parameters")

package(default_visibility = ["//visibility:public"])

cpp_dynamic_parameters(
    name = "cpp_params",
    enum_deps = [
        "//software/world:game_state",
        "//software/ai/hl/stp/play:play",
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
        "//software/parameter",
        "//software/parameter:config",
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
