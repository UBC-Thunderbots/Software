load("@bazel_rules_qt//:qt.bzl", "qt_cc_library", "qt_ui_library")

package(default_visibility = ["//visibility:public"])

qt_cc_library(
    name = "plotter_qt",
    srcs = glob(
        ["*.cpp"],
    ),
    hdrs = glob(["*.h"]),
    deps = [
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
        "@qt//:qt_opengl",
        "//extlibs/er_force_sim/src/protobuf:protobuf_cc_lib",
        ":plotter",
    ],
)

qt_ui_library(
    name = "plotter",
    ui = "plotter.ui",
    deps = [
        "@qt//:qt_widgets",
        "@qt//:qt_opengl",
    ],
)
