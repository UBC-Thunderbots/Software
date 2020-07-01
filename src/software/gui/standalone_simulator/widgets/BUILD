load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui/standalone_simulator:__subpackages__"])

qt_cc_library(
    name = "standalone_simulator_gui",
    src = "standalone_simulator_gui.cpp",
    hdr = "standalone_simulator_gui.h",
    deps = [
        "//software/gui/drawing:ssl_wrapper_packet",
        "//software/gui/standalone_simulator/ui:main_widget",
        "//software/multithreading:thread_safe_buffer",
        "//software/proto:ssl_cc_proto",
        "@qt//:qt_widgets",
    ],
)
