load("@bazel_rules_qt//:qt.bzl", "qt_ui_library")

package(default_visibility = ["//software/gui/robot_diagnostics:__subpackages__"])

qt_ui_library(
    name = "main_widget",
    ui = "main_widget.ui",
    deps = [
        "//software/gui/generic_widgets/robot_status:robot_status_table",
        "//software/gui/robot_diagnostics/widgets:decimal_progress_bar",
        "@qt//:qt_widgets",
    ],
)
