load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "gl_toolbar",
    srcs = ["gl_toolbar.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "icon_loader",
    srcs = ["icon_loader.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_field_toolbar",
    srcs = ["gl_field_toolbar.py"],
    deps = [
        ":gl_toolbar",
        "//software/thunderscope/common:common_widgets",
        requirement("pyqtgraph"),
        requirement("qtawesome"),
    ],
)

py_library(
    name = "gl_util_toolbar",
    srcs = ["gl_util_toolbar.py"],
    deps = [
        "//software/thunderscope/gl/widgets/toolbars:gl_gamecontroller_toolbar",
        "//software/thunderscope/gl/widgets/toolbars:gl_runtime_toolbar",
        ":gl_toolbar",
        "//proto:import_all_protos",
        requirement("pyqtgraph"),
        requirement("qtawesome"),
    ],
)
