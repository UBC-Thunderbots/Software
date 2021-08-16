load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//software/gui/standalone_simulator:__subpackages__"])

qt_cc_library(
    name = "standalone_simulator_draw_function_visualizer",
    srcs = ["standalone_simulator_draw_function_visualizer.cpp"],
    hdrs = ["standalone_simulator_draw_function_visualizer.h"],
    deps = [
        "//software/geom:point",
        "//software/gui:geometry_conversion",
        "//software/gui/drawing:ball",
        "//software/gui/generic_widgets/draw_function_visualizer",
        "//software/simulation:standalone_simulator",
        "//software/simulation/physics:physics_robot",
        "//software/world:robot_state",
        "//software/world:team_colour",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "standalone_simulator_gui",
    srcs = ["standalone_simulator_gui.cpp"],
    hdrs = ["standalone_simulator_gui.h"],
    deps = [
        "//software/gui/drawing:ssl_wrapper_packet",
        "//software/gui/standalone_simulator/ui:main_widget",
        "//software/proto:ssl_cc_proto",
        "//software/simulation:standalone_simulator",
        "@qt//:qt_widgets",
    ],
)
