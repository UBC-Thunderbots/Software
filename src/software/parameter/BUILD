package(default_visibility = ["//visibility:public"])

filegroup(
    name = "yaml_config_descriptions",
    srcs = glob(["config/*.yaml"]),
)

genrule(
    name = "config_cc_generate",
    srcs = [
        ":generate_parameters_v2.py",
        ":constants.py",
        "//software/parameter:yaml_config_descriptions",
    ],
    outs = [
        "generated_dynamic_parameters.hpp",
    ],
    cmd = "python3 software/parameter/generate_parameters_v2.py \
           $(locations //software/parameter:yaml_config_descriptions) \
           $(OUTS)",
    visibility = ["//visibility:private"],
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
        "generated_dynamic_parameters.hpp",
    ],
    deps = [
        ":config",
        ":parameter",
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

cc_test(
    name = "config_test",
    srcs = ["config_test.cpp"],
    data = ["//software/parameter:yaml_config_descriptions"],
    deps = [
        ":dynamic_parameters",
        ":parameter",
        "//software/util/variant_visitor",
        "@boost//:filesystem",
        "@gtest//:gtest_main",
        "@yaml-cpp",
    ],
)
