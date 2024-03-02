package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "gl_field_toolbar",
    srcs = ["gl_field_toolbar.py"],
    deps = [
        "//software/thunderscope/common:common_widgets",
        "//software/thunderscope/gl/widgets/toolbar_icons:toolbar_icon_loader",
        requirement("pyqtgraph"),
    ],
)
