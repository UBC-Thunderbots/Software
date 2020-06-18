load("@bazel_rules_qt//:qt.bzl", "qt_ui_library")

package(
    default_visibility = ["//software/gui/visualizer:__subpackages__"],
    features = ["warnings_as_errors"],
)

qt_ui_library(
    name = "main_widget",
    ui = "main_widget.ui",
    deps = ["@qt//:qt_widgets"],
)
