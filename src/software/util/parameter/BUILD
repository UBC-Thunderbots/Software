package(default_visibility = ["//visibility:public"])

filegroup(
        name = "yaml_config_descriptions",
        srcs = glob(["config/*.yaml",]),
)

genrule(
    name = "config_cc_generate",
    srcs = [
        "generate_parameters_v2.py",
        "constants.py",
        "//software/util/parameter:yaml_config_descriptions",
    ],
    outs = [
        "config.hpp",
    ],
    cmd = "python3 software/util/parameter/generate_parameters_v2.py \
           $(locations //software/util/parameter:yaml_config_descriptions) \
           $(OUTS)",
    visibility = ["//visibility:private"],
)

cc_library(
    name = "parameter",
    hdrs = ["parameter.h"],
    deps = [],
)

cc_library(
    name = "dynamic_parameters",
    hdrs = ["config.hpp", "config_utils.hpp", "dynamic_parameters.h"],
    deps = [":parameter"],
)

cc_test(
    name = "parameter_test",
    srcs = ["parameter_test.cpp"],
    deps = [
        ":parameter",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "config_test",
    srcs = ["config_test.cpp"],
    deps = [
        ":dynamic_parameters",
        ":parameter",
        "@gtest//:gtest_main",
    ],
)
