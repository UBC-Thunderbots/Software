load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_standalone_simulator_gui",
    srcs = ["threaded_standalone_simulator_gui.cpp"],
    hdrs = ["threaded_standalone_simulator_gui.h"],
    deps = [
        "//software/gui/standalone_simulator/widgets:standalone_simulator_gui",
        "//software/new_geom:rectangle",
        "//software/parameter:dynamic_parameters",
        "//software/proto:ssl_cc_proto",
        "//software/simulation:standalone_simulator",
        "@qt//:qt_widgets",
    ],
)
