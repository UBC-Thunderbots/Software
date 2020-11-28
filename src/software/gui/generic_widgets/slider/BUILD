load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "slider",
    src = "slider.cpp",
    hdr = "slider.h",
    deps = [
        "@qt//:qt_widgets",
    ],
)
