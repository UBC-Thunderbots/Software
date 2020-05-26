load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/visualizer:__subpackages__"])

qt_cc_library(
    name = "main_widget",
    src = "main_widget.cpp",
    hdr = "main_widget.h",
    deps = [
        ":ai_control",
        ":parameters",
        ":robot_status",
        ":world_view",
        "//software/ai/hl/stp:play_info",
        "//software/backend:robot_status",
        "//software/geom",
        "//software/logger",
        "//software/proto:sensor_msg_cc_proto",
        "//software/visualizer/drawing:draw_functions",
        "//software/visualizer/geom:geometry_conversion",
        "//software/visualizer/ui:main_widget",
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
        "//software/time:duration",
        "//software/visualizer/geom:geometry_conversion",
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
    name = "robot_status",
    src = "robot_status.cpp",
    hdr = "robot_status.h",
    deps = [
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "parameters",
    src = "parameters.cpp",
    hdr = "parameters.h",
    deps = [
        ":robot_status_table",
        ":zoomable_qgraphics_view",
        "//software/logger",
        "//software/parameter",
        "//software/visualizer/ui:main_widget",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "ai_control",
    src = "ai_control.cpp",
    hdr = "ai_control.h",
    deps = [
        ":robot_status_table",
        ":zoomable_qgraphics_view",
        "//software/ai/hl/stp/play",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "//software/visualizer/ui:main_widget",
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

qt_cc_library(
    name = "robot_status_table",
    src = "robot_status_table.cpp",
    hdr = "robot_status_table.h",
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software/backend:robot_status",
        "//software/time:duration",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)
