load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "robot_status",
    src = "robot_status.cpp",
    hdr = "robot_status.h",
    deps = [
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "robot_status_table",
    src = "robot_status_table.cpp",
    hdr = "robot_status_table.h",
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software/backend:robot_status",
        "//software/time:duration",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)
