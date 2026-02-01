load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "gl_runtime_installer",
    srcs = ["runtime/gl_runtime_installer.py"],
    deps = [
        "//software/thunderscope/binary_context_managers:runtime_manager",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_runtime_selector",
    srcs = ["runtime/gl_runtime_selector.py"],
    deps = [
        "//software/thunderscope/binary_context_managers:runtime_manager",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "gl_gamecontroller_toolbar",
    srcs = ["gl_gamecontroller_toolbar.py"],
    deps = [
        "//proto:import_all_protos",
        "//software/networking:ssl_proto_communication",
        "//software/thunderscope/binary_context_managers:game_controller",
        "//software/thunderscope/gl/widgets:gl_toolbar",
        requirement("pyqtgraph"),
        requirement("qtawesome"),
    ],
)

py_library(
    name = "gl_runtime_toolbar",
    srcs = ["runtime/gl_runtime_toolbar.py"],
    deps = [
        ":gl_runtime_installer",
        ":gl_runtime_selector",
        "//software/thunderscope/gl/widgets:gl_toolbar",
        "//proto:import_all_protos",
        requirement("pyqtgraph"),
        requirement("qtawesome"),
    ],
)
