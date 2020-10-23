package(default_visibility = ["//visibility:private"])

cc_library(
    name = "cpp_params_example",
    srcs = ["generated_dynamic_parameters.hpp"],
    deps = [
        "//software/ai/hl/stp",
        "//software/parameter",
        "//software/parameter:config",
        "//software/parameter:enumerated_parameter",
        "//software/parameter:numeric_parameter",
        "//software/util/design_patterns:generic_factory",
        "//software/world:game_state",
        "@boost//:program_options",
    ],
)

cc_test(
    name = "example_test",
    srcs = [
        "test.cpp",
    ],
    deps = [
        "//shared/parameter_v2/examples:cpp_params_example",
        "@gtest//:gtest_main",
    ],
)
