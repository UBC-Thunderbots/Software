load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "gl_runtime_installer",
    srcs = ["gl_runtime_installer.py"],
    deps = [
        "//software/thunderscope/binary_context_managers:runtime_manager",
        requirement("pyqtgraph"),
    ],
)

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
    name = "gl_gamecontroller_toolbar",
    srcs = ["gl_gamecontroller_toolbar.py"],
    deps = [
        ":gl_runtime_installer",
        ":gl_toolbar",
        "//proto:import_all_protos",
        "//software/networking:ssl_proto_communication",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/common:common_widgets",
        requirement("pyqtgraph"),
        requirement("qtawesome"),
    ],
)
