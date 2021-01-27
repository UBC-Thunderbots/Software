load("@rules_python//python:defs.bzl", "py_binary")
load("@dynamic_parameter_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:private"])

py_binary(
    name = "generate_dynamic_parameters",
    srcs = [
        ":c_config.py",
        ":c_parameter.py",
        ":c_writer.py",
        ":cpp_config.py",
        ":cpp_parameter.py",
        ":cpp_writer.py",
        ":generate_dynamic_parameters.py",
        ":type_map.py",
    ],
    data = [
        "//shared/parameter_v2/config_definitions:test_configs",
    ],
    main = ":generate_dynamic_parameters.py",
    python_version = "PY3",
    visibility = ["//shared/parameter_v2:__subpackages__"],
    deps = [
        requirement("PyYAML"),
        requirement("jsonschema"),
        requirement("colorama"),
        requirement("networkx"),
        requirement("decorator"),
    ],
)
