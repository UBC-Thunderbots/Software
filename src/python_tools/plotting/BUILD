package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_library(
    name = "plot_ssl_wrapper",
    srcs = ["plot_ssl_wrapper.py"],
    deps = [
        requirement("bokeh"),
        requirement("numpy"),
        "//software/proto:software_py_proto",
    ],
)

py_library(
    name = "plot_heatmap",
    srcs = ["plot_heatmap.py"],
    deps = [
        requirement("bokeh"),
        requirement("numpy"),
    ],
)
