package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_full_system_gui",
    srcs = ["threaded_full_system_gui.cpp"],
    hdrs = ["threaded_full_system_gui.h"],
    deps = [
        "//proto:play_info_msg_cc_proto",
        "//proto:sensor_msg_cc_proto",
        "//proto:tbots_cc_proto",
        "//proto:visualization_cc_proto",
        "//proto/message_translation:tbots_protobuf",
        "//software/gui/drawing:draw_functions",
        "//software/gui/drawing:world",
        "//software/gui/full_system/widgets:full_system_gui",
        "//software/multithreading:thread_safe_buffer",
        "//software/multithreading:threaded_observer",
        "//software/world",
        "@qt//:qt_widgets",
    ],
)
