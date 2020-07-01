load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "main_widget",
    src = "main_widget.cpp",
    hdr = "main_widget.h",
    deps = [
        ":ai_control",
        ":parameters",
        "//software/ai/hl/stp:play_info",
        "//software/backend:robot_status",
        "//software/geom",
        "//software/gui:geometry_conversion",
        "//software/gui/drawing:draw_functions",
        "//software/gui/generic_widgets/robot_status",
        "//software/gui/generic_widgets/world_view",
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
        "//software/gui:geometry_conversion",
        "//software/multithreading:thread_safe_buffer",
        "//software/proto:sensor_msg_cc_proto",
        "//software/time:duration",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "parameters",
    src = "parameters.cpp",
    hdr = "parameters.h",
    deps = [
        "//software/gui/visualizer/ui:main_widget",
        "//software/logger",
        "//software/parameter",
        "//software/parameter:config",
        "//software/parameter:dynamic_parameters",
        "//software/util/variant_visitor",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "ai_control",
    src = "ai_control.cpp",
    hdr = "ai_control.h",
    deps = [
        "//software/ai/hl/stp/play",
        "//software/gui/visualizer/ui:main_widget",
        "//software/parameter:dynamic_parameters",
        "//software/util/design_patterns:generic_factory",
        "@qt//:qt_widgets",
    ],
)
