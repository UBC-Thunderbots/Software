load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "icon_loader",
    srcs = ["icon_loader.py"],
    data = [
        ":bookmark.svg",
        ":help.svg",
        ":measure.svg",
        ":pause.svg",
        ":play.svg",
        ":redo.svg",
        ":reset.svg",
        ":undo.svg",
        ":view.svg",
        "//software/thunderscope/gl/widgets:icon_loader",
    ],
    deps = [
        requirement("pyqtgraph"),
    ],
)
