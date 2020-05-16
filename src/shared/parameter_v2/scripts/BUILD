load("@rules_python//python:defs.bzl", "py_binary")
load("@dynamic_parameter_deps//:requirements.bzl", "requirement")

py_binary(
    name = "generate_dynamic_parameters",
    srcs = [":generate_dynamic_parameters.py"],
    data = [
        "//shared/parameter_v2/config:example_configs",
    ],
    main = ":generate_dynamic_parameters.py",
    visibility = ["//shared/parameter_v2:__subpackages__"],
    deps = [
        requirement("PyYAML"),
    ],
)
