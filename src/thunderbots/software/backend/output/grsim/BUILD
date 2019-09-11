package(default_visibility = ["//visibility:public"])

cc_library(
    name = "grsim_command_primitive_visitor",
    srcs = ["grsim_command_primitive_visitor.cpp"],
    hdrs = ["grsim_command_primitive_visitor.h"],
    deps = [
        ":motion_controller",
        "//shared:constants",
        "//software/ai/primitive:all_primitives",
        "//software/ai/primitive:primitive_visitor",
        "//software/ai/world:ball",
        "//software/ai/world:robot",
    ],
)

cc_library(
    name = "grsim_output",
    srcs = ["grsim_output.cpp"],
    hdrs = ["grsim_output.h"],
    deps = [
        ":grsim_command_primitive_visitor",
        "//shared:constants",
        "//software/ai/world:team",
        "//software/proto:grsim_cc_proto",
        "//software/util/parameter:dynamic_parameters",
    ],
)

cc_library(
    name = "motion_controller",
    srcs = ["motion_controller.cpp"],
    hdrs = ["motion_controller.h"],
    deps = [
        "//software/ai/world:robot",
    ],
)
