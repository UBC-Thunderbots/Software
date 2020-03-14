load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/robot_diagnostics:__subpackages__"])

qt_cc_library(
    name = "robot_diagnostics_widget",
    src = "main_widget.cpp",
    hdr = "main_widget.h",
    deps = [
        ":feedback",
        "//software/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "feedback",
    src = "feedback.cpp",
    hdr = "feedback.h",
    deps = [
        "//software/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)
