load(":parameter.bzl", "cpp_dynamic_parameters", "proto_dynamic_parameters")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")
load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cpp_dynamic_parameters(
    name = "cpp_params",
    enum_deps = [
        "//software/world:game_state",
        "//software/world:team_colour",
    ],
    generated_parameter_header = "cpp_dynamic_parameters.h",
    generated_parameter_source = "cpp_dynamic_parameters.cpp",
)

proto_dynamic_parameters(
    name = "proto_params",
    generated_proto_source = "dynamic_parameters.proto",
)

proto_library(
    name = "dynamic_param_proto",
    srcs = [
        ":proto_params",
    ],
    deps = [
        "@com_google_protobuf//:any_proto",
        "@nanopb//:nanopb_options_proto",
    ],
)

cc_proto_library(
    name = "dynamic_param_cc_proto",
    deps = [":dynamic_param_proto"],
)

py_proto_library(
    name = "dynamic_param_py_proto",
    srcs = [
        "dynamic_parameters.proto",
    ],
    deps = [
        "@com_google_protobuf//:protobuf_python",
    ],
)

cc_library(
    name = "cpp_configs",
    srcs = [
        ":cpp_params",
    ],
    hdrs = [
        ":cpp_params",
    ],
    deps = [
        ":config",
        ":dynamic_param_cc_proto",
        ":enumerated_parameter",
        ":numeric_parameter",
        ":parameter",
        "//software/util/generic_factory",
        "//software/world:game_state",
        "//software/world:team_colour",
    ],
)

cc_test(
    name = "cpp_config_test",
    srcs = [
        "cpp_config_test.cpp",
    ],
    data = [
        "//shared/parameter/config_definitions:parameter_configs",
    ],
    deps = [
        "//shared/parameter:cpp_configs",
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/play:halt_play",
        "//software/util/variant_visitor",
        "@boost//:filesystem",
        "@yaml-cpp",
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

cc_test(
    name = "parameter_test",
    srcs = ["parameter_test.cpp"],
    deps = [
        ":enumerated_parameter",
        ":numeric_parameter",
        ":parameter",
        "//shared/test_util:tbots_gtest_main",
        "//software/util/make_enum",
    ],
)

PYBIND_DEPS = [
    "@local_config_python//:python_headers",
    "@pybind11",
]

cc_library(
    name = "pyton_bindings_lib",
    srcs = ["python_bindings.cpp"],
    copts = ["-fexceptions"],
    features = [
        "-use_header_modules",
        "-parse_headers",
    ],
    deps = [
        "//software/sensor_fusion",
        "//shared/parameter:cpp_configs",
        "//proto:sensor_msg_cc_proto",
        "//software/simulation:er_force_simulator",
        "@pybind11_protobuf//pybind11_protobuf:native_proto_caster",
    ] + PYBIND_DEPS,
)

pybind_extension(
    name = "python_bindings",
    deps = [
        ":pyton_bindings_lib",
    ],
)

py_test(
    name = "simulated_test_fixture",
    srcs = ["simulated_test_fixture.py"],
    data = [
        ":python_bindings.so",
    ],
    deps = [
        "//proto:software_py_proto",
    ],
)
