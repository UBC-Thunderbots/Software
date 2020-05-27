load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/qt_gui/visualizer:__subpackages__"])

qt_cc_library(
    name = "main_widget",
    src = "main_widget.cpp",
    hdr = "main_widget.h",
    deps = [
        ":ai_control",
        ":parameters",
        ":world_view",
        "//software/ai/hl/stp:play_info",
        "//software/backend:robot_status",
        "//software/geom",
        "//software/logger",
        "//software/proto:sensor_msg_cc_proto",
        "//software/qt_gui/generic_widgets:robot_status",
        "//software/qt_gui/visualizer/drawing:draw_functions",
        "//software/qt_gui/visualizer/geom:geometry_conversion",
        "//software/qt_gui/visualizer/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "visualizer",
    src = "visualizer.cpp",
    hdr = "visualizer.h",
    deps = [
        ":main_widget",
        "//software/ai/hl/stp:play_info",
        "//software/backend:robot_status",
        "//software/multithreading:thread_safe_buffer",
        "//software/proto:sensor_msg_cc_proto",
        "//software/qt_gui/visualizer/geom:geometry_conversion",
        "//software/time:duration",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "world_view",
    src = "world_view.cpp",
    hdr = "world_view.h",
    deps = [
        ":zoomable_qgraphics_view",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "parameters",
    src = "parameters.cpp",
    hdr = "parameters.h",
    deps = [
        ":zoomable_qgraphics_view",
        "//software/logger",
        "//software/parameter",
        "//software/qt_gui/generic_widgets:robot_status_table",
        "//software/qt_gui/visualizer/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "ai_control",
    src = "ai_control.cpp",
    hdr = "ai_control.h",
    deps = [
        ":zoomable_qgraphics_view",
        "//software/ai/hl/stp/play",
        "//software/parameter:dynamic_parameters",
        "//software/qt_gui/generic_widgets:robot_status_table",
        "//software/qt_gui/visualizer/ui:main_widget",
        "//software/util/design_patterns:generic_factory",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "zoomable_qgraphics_view",
    src = "zoomable_qgraphics_view.cpp",
    hdr = "zoomable_qgraphics_view.h",
    deps = [
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)
