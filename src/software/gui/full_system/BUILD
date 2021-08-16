package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_full_system_gui",
    srcs = ["threaded_full_system_gui.cpp"],
    hdrs = ["threaded_full_system_gui.h"],
    deps = [
        "//software/ai/hl/stp:play_info",
        "//software/gui/drawing:draw_functions",
        "//software/gui/drawing:world",
        "//software/gui/full_system/widgets:full_system_gui",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/proto:sensor_msg_cc_proto",
        "//software/world",
        "@qt//:qt_widgets",
    ],
)
