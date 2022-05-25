package(default_visibility = ["//visibility:private"])

filegroup(
    name = "test_configs",
    srcs = glob([
        "test/**/*.yaml",
    ]),
    visibility = ["//shared/parameter:__subpackages__"],
)

filegroup(
    name = "parameter_configs",
    srcs = glob(
        ["**/*.yaml"],
        exclude = ["test/**/*.yaml"],
    ),
    visibility = ["//shared/parameter:__subpackages__"],
)
