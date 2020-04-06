load("@bazel_rules_qt//:qt.bzl", "qt_ui_library")

package(default_visibility = ["//software/robot_diagnostics:__subpackages__"])

qt_ui_library(
    name = "main_widget",
    ui = "main_widget.ui",
    deps = ["@qt//:qt_widgets"],
)
