package(default_visibility = ["//visibility:public"])

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")

cc_library(
    name = "python_binding_utilities",
    srcs = [
        "python_binding_utilities.cpp",
    ],
    hdrs = [
        "python_binding_utilities.h",
    ],
    deps = [
        "//software/parameter:config",
        "//software/util/variant_visitor",
        "@local_config_python//:python_headers",
        "@pybind11",
    ],
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
        "//software/parameter:dynamic_parameters",
        "//software/proto:sensor_msg_cc_proto",
        "//software/proto:ssl_cc_proto",
        "//software/sensor_fusion",
        "@local_config_python//:python_headers",
        "@pybind11",
    ],
)

pybind_extension(
    name = "passing",
    deps = [
        ":passing_lib",
    ],
)
