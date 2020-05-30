load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui/visualizer:__subpackages__"])

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
        "//software/gui/generic_widgets/robot_status",
        "//software/gui/visualizer/drawing:draw_functions",
        "//software/gui/visualizer/geom:geometry_conversion",
        "//software/gui/visualizer/ui:main_widget",
        "//software/logger",
        "//software/proto:sensor_msg_cc_proto",
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
        "//software/gui/visualizer/geom:geometry_conversion",
        "//software/multithreading:thread_safe_buffer",
        "//software/proto:sensor_msg_cc_proto",
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
        "//software/gui/generic_widgets/robot_status:robot_status_table",
        "//software/gui/visualizer/ui:main_widget",
        "//software/logger",
        "//software/parameter",
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
        "//software/gui/generic_widgets/robot_status:robot_status_table",
        "//software/gui/visualizer/ui:main_widget",
        "//software/parameter:dynamic_parameters",
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
