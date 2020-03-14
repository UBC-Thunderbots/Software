package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "robot_diagnostics",
    srcs = ["robot_diagnostics_main.cpp"],
    deps = [
        "//software/robot_diagnostics/widgets:robot_diagnostics_widget",
        "@boost//:program_options",
        "@g3log",
    ],
)
