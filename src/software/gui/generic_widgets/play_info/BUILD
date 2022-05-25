load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "play_info_widget",
    srcs = ["play_info_widget.cpp"],
    hdrs = ["play_info_widget.h"],
    deps = [
        "//proto:play_info_msg_cc_proto",
        "@qt//:qt_widgets",
    ],
)
