package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_library(
    name = "ssl_wrapper_plot_helpers",
    srcs = ["ssl_wrapper_plot_helpers.py"],
    deps = [
        requirement("bokeh"),
        requirement("numpy"),
        "//software/proto:software_py_proto",
    ],
)
