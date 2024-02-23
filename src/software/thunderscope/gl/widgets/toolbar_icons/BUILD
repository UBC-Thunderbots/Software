package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "toolbar_icon_loader",
    srcs = ["toolbar_icon_loader.py"],
    data = [
        ":help.svg",
        ":measure.svg",
        ":pause.svg",
        ":play.svg",
        ":redo.svg",
        ":reset.svg",
        ":undo.svg",
        ":view.svg",
    ],
    deps = [
        requirement("pyqtgraph"),
    ],
)
