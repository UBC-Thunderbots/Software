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
    name = "gl_runtime_selector",
    srcs = ["gl_runtime_selector.py"],
    deps = [
        "//software/thunderscope/binary_context_managers:runtime_manager",
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
