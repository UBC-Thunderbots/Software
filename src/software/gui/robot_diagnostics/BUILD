package(default_visibility = ["//visibility:public"])

cc_library(
    name = "robot_diagnostics_wrapper",
    srcs = ["robot_diagnostics_wrapper.cpp"],
    hdrs = ["robot_diagnostics_wrapper.h"],
    deps = [
        "//software/geom",
        "//software/gui/robot_diagnostics/widgets:robot_diagnostics_widget",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/primitive",
        "//software/proto:sensor_msg_cc_proto",
        "//software/world",
        "@qt//:qt_widgets",
    ],
)

cc_binary(
    name = "robot_diagnostics_main",
    srcs = ["robot_diagnostics_main.cpp"],
    deps = [
        ":robot_diagnostics_wrapper",
        "//software/logger",
        "@boost//:program_options",
    ],
)
