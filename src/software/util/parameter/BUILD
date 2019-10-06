package(default_visibility = ["//visibility:public"])

genrule(
    name = "dynamic_parameters_cc_generate",
    srcs = [
        "generate_parameters.py",
        "constants.py",
        "config/ai.yaml",
        "config/ai_control.yaml",
        "config/enemy_team_capability.yaml",
        "config/particle_filter.yaml",
        "config/vision_and_filters.yaml",
        "config/xbox_controller.yaml",
    ],
    outs = [
        # NOTE: the order of these matters, since we give them as command line
        #       arguments to the generation file
        "dynamic_parameters.h",
        "dynamic_parameters.cpp",
    ],
    cmd = "python3 software/util/parameter/generate_parameters.py $(OUTS)",
    visibility = ["//visibility:private"],
)

cc_library(
    name = "parameter",
    hdrs = ["parameter.h"],
    deps = [],
)

cc_library(
    name = "config",
    hdrs = ["config.hpp"],
    deps = [],
)

cc_test(
    name = "parameter_test",
    srcs = ["parameter_test.cpp"],
    deps = [
        ":parameter",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "dynamic_parameters",
    srcs = ["dynamic_parameters.cpp"],
    hdrs = [
        "dynamic_parameters.h",
    ],
    deps = [":parameter"],
)

cc_test(
    name = "config_test",
    srcs = ["config_test.cpp"],
    deps = [
        ":config",
        ":parameter",
        "@gtest//:gtest_main",
    ],
)
