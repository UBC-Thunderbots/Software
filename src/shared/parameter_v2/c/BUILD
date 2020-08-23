cc_library(
    name = "parameter",
    srcs = ["parameter.c"],
    hdrs = ["parameter.h"],
)

cc_library(
    name = "example_config",
    srcs = ["example_config.c"],
    hdrs = ["example_config.h"],
    deps = [":parameter"],
)

genrule(
    name = "config_c_generate",
    srcs = [],
    outs = [
        "tbots_c_config.h",
        "tbots_c_config.c",
    ],
    cmd = "$(location //shared/parameter_v2/scripts:generate_dynamic_parameters)" +
          " --output tbots_c_config -c " +
          "&& mv tbots_c_config.h $(location tbots_c_config.h)" +
          "&& mv tbots_c_config.c $(location tbots_c_config.c)",
    tools = ["//shared/parameter_v2/scripts:generate_dynamic_parameters"],
)

cc_library(
    name = "config",
    srcs = ["tbots_c_config.c"],
    hdrs = ["tbots_c_config.h"],
    deps = [":parameter"],
)
