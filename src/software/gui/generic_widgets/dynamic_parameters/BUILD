load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "dynamic_parameter_widget",
    srcs = ["dynamic_parameter_widget.cpp"],
    hdrs = ["dynamic_parameter_widget.h"],
    deps = [
        "//shared/parameter",
        "//shared/parameter:cpp_configs",
        "//software/gui/generic_widgets/slider",
        "//software/gui/shared:parameters_spinbox",
        "//software/logger",
        "//software/util/variant_visitor",
        "@qt//:qt_widgets",
    ],
)
