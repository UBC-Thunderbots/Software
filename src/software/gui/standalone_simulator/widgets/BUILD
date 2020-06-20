load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui/standalone_simulator:__subpackages__"])

qt_cc_library(
    name = "main_widget",
    src = "main_widget.cpp",
    hdr = "main_widget.h",
    deps = [
        "//software/gui:geometry_conversion",
        "//software/gui/drawing:draw_functions",
        "//software/gui/generic_widgets/world_view",
        "//software/gui/standalone_simulator/ui:main_widget",
        "//software/logger",
        "//software/proto:ssl_cc_proto",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "simulator_gui",
    src = "standalone_simulator_gui.cpp",
    hdr = "standalone_simulator_gui.h",
    deps = [
        ":main_widget",
        "//software/gui:geometry_conversion",
        "//software/gui/drawing:ssl_wrapper_packet",
        "//software/multithreading:thread_safe_buffer",
        "//software/proto:ssl_cc_proto",
        "@qt//:qt_widgets",
    ],
)
