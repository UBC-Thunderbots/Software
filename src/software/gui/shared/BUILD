package(default_visibility = ["//software/gui:__subpackages__"])

cc_library(
    name = "parameters_spinbox",
    srcs = ["parameters_spinbox.cpp"],
    hdrs = ["parameters_spinbox.h"],
    deps = [
        "//software/logger",
        "//shared/parameter_v2:parameter",
        "//shared/parameter_v2:cpp_configs",
        "//software/util/variant_visitor",
        "@qt//:qt_widgets",
    ],
)
