load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui/robot_diagnostics:__subpackages__"])

qt_cc_library(
    name = "robot_diagnostics_widget",
    src = "main_widget.cpp",
    hdr = "main_widget.h",
    deps = [
        ":chicker",
        ":dribbler",
        ":drive",
        ":feedback",
        ":leds",
        ":robot_selection",
        "//software/gui/generic_widgets/robot_status",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "chicker",
    src = "chicker.cpp",
    hdr = "chicker.h",
    deps = [
        ":slider",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "dribbler",
    src = "dribbler.cpp",
    hdr = "dribbler.h",
    deps = [
        ":slider",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "drive",
    src = "drive.cpp",
    hdr = "drive.h",
    deps = [
        ":slider",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "feedback",
    src = "feedback.cpp",
    hdr = "feedback.h",
    deps = [
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "leds",
    src = "leds.cpp",
    hdr = "leds.h",
    deps = [
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "robot_selection",
    src = "robot_selection.cpp",
    hdr = "robot_selection.h",
    deps = [
        "//shared:constants",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "//software/logger",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "slider",
    src = "slider.cpp",
    hdr = "slider.h",
    deps = [
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)
