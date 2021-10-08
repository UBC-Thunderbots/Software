load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    srcs = glob(
        ["*.cpp"],
    ),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        ":core_qt",
    ],
)

qt_cc_library(
    name = "core_qt",
    srcs = glob(
        ["*.cpp"],
    ),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)
