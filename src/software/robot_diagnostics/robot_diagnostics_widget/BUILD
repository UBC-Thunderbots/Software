load("@bazel_rules_qt//:qt.bzl", "qt_ui_library", "qt_cc_library")

package(default_visibility = ["//software/visualizer:__subpackages__"])

qt_ui_library(
    name = "main_widget",
    ui = "mainwidget.ui",
    deps = ["@qt//:qt_widgets"],
)

qt_cc_library(
    name = "robot_diagnostics_widget",
    src = "mainwidget.cpp",
    hdr = "mainwidget.h",
    deps = [
        ":main_widget",
        "@qt//:qt_widgets",
    ],
)

