package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_standalone_simulator_gui",
    srcs = ["threaded_standalone_simulator_gui.cpp"],
    hdrs = ["threaded_standalone_simulator_gui.h"],
    deps = [
        "//shared/parameter:cpp_configs",
        "//software/geom:rectangle",
        "//software/gui/standalone_simulator/widgets:standalone_simulator_gui",
        "//software/proto:ssl_cc_proto",
        "//software/simulation:standalone_simulator",
        "@qt//:qt_widgets",
    ],
)
