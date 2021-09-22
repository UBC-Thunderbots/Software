load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui/robot_diagnostics:__subpackages__"])

qt_cc_library(
    name = "robot_diagnostics_gui",
    srcs = ["robot_diagnostics_gui.cpp"],
    hdrs = ["robot_diagnostics_gui.h"],
    deps = [
        ":chicker",
        ":dribbler",
        ":drive",
        ":leds",
        ":robot_selection",
        ":sensor_status",
        "//proto:sensor_msg_cc_proto",
        "//proto/message_translation:tbots_geometry",
        "//proto/message_translation:tbots_protobuf",
        "//software/gui/generic_widgets/robot_status",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "//software/multithreading:thread_safe_buffer",
        "@boost//:bind",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "chicker",
    srcs = ["chicker.cpp"],
    hdrs = ["chicker.h"],
    deps = [
        "//software/gui/generic_widgets/slider",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "//software/util/make_enum",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "dribbler",
    srcs = ["dribbler.cpp"],
    hdrs = ["dribbler.h"],
    deps = [
        "//shared:constants",
        "//shared:robot_constants",
        "//software/gui/generic_widgets/slider",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "drive",
    srcs = ["drive.cpp"],
    hdrs = ["drive.h"],
    deps = [
        "//software/gui/generic_widgets/slider",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "//software/util/make_enum",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "sensor_status",
    srcs = ["sensor_status.cpp"],
    hdrs = ["sensor_status.h"],
    deps = [
        ":decimal_progress_bar",
        "//proto:sensor_msg_cc_proto",
        "//shared:constants",
        "//shared:robot_constants",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "leds",
    srcs = ["leds.cpp"],
    hdrs = ["leds.h"],
    deps = [
        "//software/gui/robot_diagnostics/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "robot_selection",
    srcs = ["robot_selection.cpp"],
    hdrs = ["robot_selection.h"],
    deps = [
        "//shared:constants",
        "//shared:robot_constants",
        "//software/gui/robot_diagnostics/ui:main_widget",
        "//software/logger",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "decimal_progress_bar",
    srcs = ["decimal_progress_bar.cpp"],
    hdrs = ["decimal_progress_bar.h"],
    deps = [
        "@qt//:qt_widgets",
    ],
)
