load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui:__subpackages__"])

qt_cc_library(
    name = "full_system_gui",
    srcs = ["full_system_gui.cpp"],
    hdrs = ["full_system_gui.h"],
    deps = [
        ":ai_control",
        "//proto:play_info_msg_cc_proto",
        "//proto:sensor_msg_cc_proto",
        "//software/gui:geometry_conversion",
        "//software/gui/full_system/ui:main_widget",
        "//software/gui/generic_widgets/robot_status",
        "//software/multithreading:thread_safe_buffer",
        "//software/time:duration",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "ai_control",
    srcs = ["ai_control.cpp"],
    hdrs = ["ai_control.h"],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/ai/hl/stp/play",
        "//software/gui/full_system/ui:main_widget",
        "//software/gui/shared:parameters_spinbox",
        "//software/util/generic_factory",
        "@qt//:qt_widgets",
    ],
)
