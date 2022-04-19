package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cpp_configs",
    srcs = [
        "cpp_dynamic_parameters.cpp",
    ],
    hdrs = [
        "cpp_dynamic_parameters.h",
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
