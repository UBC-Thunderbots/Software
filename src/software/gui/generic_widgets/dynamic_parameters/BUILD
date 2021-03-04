load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "dynamic_parameter_widget",
    src = "dynamic_parameter_widget.cpp",
    hdr = "dynamic_parameter_widget.h",
    deps = [
        "//shared/parameter:cpp_configs",
        "//shared/parameter:parameter",
        "//software/gui/generic_widgets/slider",
        "//software/gui/shared:parameters_spinbox",
        "//software/logger",
        "//software/util/variant_visitor",
        "@qt//:qt_widgets",
    ],
)
