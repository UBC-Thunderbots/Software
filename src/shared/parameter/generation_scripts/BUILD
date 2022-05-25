load("@rules_python//python:defs.bzl", "py_binary")
load("@dynamic_parameter_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:private"])

py_binary(
    name = "generate_cpp_dynamic_parameters",
    srcs = [
        ":case_conversion.py",
        ":cpp_header_config.py",
        ":cpp_parameter.py",
        ":cpp_source_config.py",
        ":cpp_writer.py",
        ":generate_cpp_dynamic_parameters.py",
        ":type_map.py",
    ],
    data = [
        "//shared/parameter/config_definitions:parameter_configs",
    ],
    main = ":generate_cpp_dynamic_parameters.py",
    python_version = "PY3",
    visibility = ["//shared/parameter:__subpackages__"],
    deps = [
        requirement("PyYAML"),
        requirement("jsonschema"),
        requirement("colorama"),
        requirement("networkx"),
        requirement("decorator"),
    ],
)

py_binary(
    name = "generate_proto_dynamic_parameters",
    srcs = [
        ":generate_proto_dynamic_parameters.py",
        ":proto_writer.py",
        ":type_map.py",
    ],
    data = [
        "//shared/parameter/config_definitions:parameter_configs",
    ],
    main = ":generate_proto_dynamic_parameters.py",
    python_version = "PY3",
    visibility = ["//shared/parameter:__subpackages__"],
    deps = [
        requirement("PyYAML"),
        requirement("jsonschema"),
        requirement("colorama"),
        requirement("networkx"),
        requirement("decorator"),
    ],
)
