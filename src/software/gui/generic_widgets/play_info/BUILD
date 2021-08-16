load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "play_info_widget",
    srcs = ["play_info_widget.cpp"],
    hdrs = ["play_info_widget.h"],
    deps = [
        "//software/ai/hl/stp:play_info",
        "@qt//:qt_widgets",
    ],
)
