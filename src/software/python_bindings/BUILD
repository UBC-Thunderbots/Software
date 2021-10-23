package(default_visibility = ["//visibility:public"])

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")

PYBIND_DEPS = [
    "@local_config_python//:python_headers",
    "@pybind11",
]

cc_library(
    name = "python_binding_utilities",
    srcs = [
        "python_binding_utilities.cpp",
    ],
    hdrs = [
        "python_binding_utilities.h",
    ],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/util/variant_visitor",
    ] + PYBIND_DEPS,
)

cc_library(
    name = "passing_lib",
    srcs = ["passing.cpp"],
    copts = ["-fexceptions"],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    deps = [
        ":python_binding_utilities",
        "//software/ai/passing:cost_functions",
        "//shared/parameter:cpp_configs",
        "//proto:sensor_msg_cc_proto",
        "//proto:ssl_cc_proto",
        "//software/sensor_fusion",
        ":pass_utilities",
    ] + PYBIND_DEPS,
)

pybind_extension(
    name = "passing",
    deps = [
        ":passing_lib",
    ],
)

cc_library(
    name = "world_lib",
    srcs = ["world.cpp"],
    copts = ["-fexceptions"],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    deps = [
        ":python_binding_utilities",
        "//software/geom:angle",
        "//software/geom:point",
        "//software/geom:vector",
        "//shared/parameter:cpp_configs",
        "//proto:sensor_msg_cc_proto",
        "//proto:ssl_cc_proto",
        "//software/sensor_fusion",
        "//software/world",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:robot",
    ] + PYBIND_DEPS,
)

pybind_extension(
    name = "world",
    deps = [
        ":world_lib",
    ],
)

cc_library(
    name = "pass_utilities",
    srcs = [
        "pass_utilities.cpp",
    ],
    hdrs = [
        "pass_utilities.h",
    ],
    deps = [
        "//software/ai/passing:pass",
    ] + PYBIND_DEPS,
)

cc_library(
    name = "pass_generator_lib",
    srcs = ["pass_generator.cpp"],
    copts = ["-fexceptions"],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    deps = [
        ":python_binding_utilities",
        ":pass_utilities",
        "//software/geom:angle",
        "//software/geom:point",
        "//software/geom:vector",
        "//shared/parameter:cpp_configs",
        "//proto:sensor_msg_cc_proto",
        "//proto:ssl_cc_proto",
        "//software/sensor_fusion",
        "//software/world",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:robot",
        "//software/ai/passing:pass_generator",
        "//software/geom:rectangle",
    ] + PYBIND_DEPS,
)

pybind_extension(
    name = "pass_generator",
    deps = [
        ":pass_generator_lib",
    ],
)

cc_library(
    name = "keep_away_lib",
    srcs = ["keep_away.cpp"],
    copts = ["-fexceptions"],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    deps = [
        ":python_binding_utilities",
        ":pass_utilities",
        "//software/ai/evaluation:keep_away",
        "//software/math:math_functions",
    ] + PYBIND_DEPS,
)

pybind_extension(
    name = "keep_away",
    deps = [
        ":keep_away_lib",
    ],
)
