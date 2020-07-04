load("@rules_python//python:defs.bzl", "py_binary")
load("@dynamic_parameter_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:private"])

py_binary(
    name = "generate_dynamic_parameters",
    srcs = [":generate_dynamic_parameters.py"],
    data = [
        "//shared/parameter_v2/config:example_configs",
    ],
    main = ":generate_dynamic_parameters.py",
    deps = [
        requirement("PyYAML"),
        requirement("jsonschema"),
        requirement("colorama"),
        requirement("networkx"),
    ],
)
