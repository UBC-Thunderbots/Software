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
        "//software/ai/passing:evaluation",
        "//shared/parameter:cpp_configs",
        "//software/proto:sensor_msg_cc_proto",
        "//software/proto:ssl_cc_proto",
        "//software/sensor_fusion",
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
        "//software/proto:sensor_msg_cc_proto",
        "//software/proto:ssl_cc_proto",
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
