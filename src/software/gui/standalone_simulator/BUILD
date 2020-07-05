load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_standalone_simulator_gui",
    srcs = ["threaded_standalone_simulator_gui.cpp"],
    hdrs = ["threaded_standalone_simulator_gui.h"],
    deps = [
        "//software/gui/standalone_simulator/widgets:standalone_simulator_gui",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/new_geom:rectangle",
        "//software/proto:ssl_cc_proto",
        "//software/proto/message_translation:ssl_geometry",
        "@qt//:qt_widgets",
    ],
)
