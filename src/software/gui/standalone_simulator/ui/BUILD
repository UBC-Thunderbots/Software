load("@bazel_rules_qt//:qt.bzl", "qt_ui_library")

package(default_visibility = ["//software/gui/standalone_simulator:__subpackages__"])

qt_ui_library(
    name = "main_widget",
    ui = "main_widget.ui",
    deps = [
        # Need to include targets used in promoted widgets
        "//software/gui/generic_widgets/draw_function_visualizer",
        "@qt//:qt_widgets",
    ],
)
