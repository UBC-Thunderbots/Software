package(default_visibility = ["//visibility:private"])

filegroup(
    name = "example_configs",
    srcs = glob([
        "examples/**/*.yaml",
    ]),
    visibility = ["//shared/parameter_v2:__subpackages__"],
)

filegroup(
    name = "parameter_configs",
    srcs = glob(
        ["**/*.yaml"],
        exclude = ["examples/**/*.yaml"],
    ),
    visibility = ["//shared/parameter_v2:__subpackages__"],
)
