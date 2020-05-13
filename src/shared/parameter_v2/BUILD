load(":parameter.bzl", "c_dynamic_parameters", "cpp_dynamic_parameters")

cpp_dynamic_parameters(
    name = "cpp_params",
    enum_targets = [
        "//software/sensor_fusion:refbox_data",
    ],
    generated_parameter_file = "cpp_dynamic_parameters.h",
)

c_dynamic_parameters(
    name = "c_params",
    generated_parameter_file = "c_parameters.h",
)
