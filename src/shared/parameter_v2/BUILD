load(":parameter.bzl", "generate_dynamic_parameters")

generate_dynamic_parameters(
    name = "dynamic_parameters_cpp",
    enum_targets = [
        "//software/sensor_fusion:refbox_data",
    ],
    generated_parameter_file = "dynamic_parameters.hpp",
)

generate_dynamic_parameters(
    name = "dynamic_parameters_c",
    enum_targets = [
        "//software/sensor_fusion:refbox_data",
    ],
    generated_parameter_file = "dynamic_parameters.h",
)
