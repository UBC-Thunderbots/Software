load("@thunderscope_deps//:requirements.bzl", "requirement")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "gl_sandbox_sidebar",
    srcs = ["gl_sandbox_sidebar.py"],
    deps = [
        "//software/thunderscope/common:common_widgets",
        requirement("pyqtgraph"),
        requirement("qtawesome"),
    ],
)

py_library(
    name = "local_robot_state_provider",
    srcs = ["local_robot_state_provider.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)
