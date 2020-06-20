load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_gui_wrapper",
    srcs = ["simulator_gui_wrapper.cpp"],
    hdrs = ["simulator_gui_wrapper.h"],
    deps = [
        "//software/gui/simulator/widgets:simulator_gui",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/new_geom:rectangle",
        "//software/proto:ssl_cc_proto",
        "//software/proto/message_translation:ssl_geometry",
        "@qt//:qt_widgets",
    ],
)

cc_binary(
    name = "simulator_gui_main",
    srcs = ["simulator_main.cpp"],
    deps = [
        ":simulator_gui_wrapper",
        "//software/logger",
        "//software/simulation:threaded_simulator",
    ],
)
