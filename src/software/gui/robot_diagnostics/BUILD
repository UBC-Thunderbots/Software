package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_robot_diagnostics_gui",
    srcs = ["threaded_robot_diagnostics_gui.cpp"],
    hdrs = ["threaded_robot_diagnostics_gui.h"],
    deps = [
        "//proto:sensor_msg_cc_proto",
        "//software/gui/robot_diagnostics/widgets:robot_diagnostics_gui",
        "//software/multithreading:subject",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/world",
        "@qt//:qt_widgets",
    ],
)
