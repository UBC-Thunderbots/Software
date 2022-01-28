package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

NOTEBOOK_DEPS = [
    requirement("jupyter"),
    requirement("notebook"),
    requirement("numpy"),
    requirement("matplotlib"),
    requirement("bokeh"),
    requirement("ipywidgets"),
    requirement("jupytext"),
    ":proto_log",
    ":sensor_proto_log",
    "//python_tools/plotting:plot_heatmap",
    "//python_tools/plotting:plot_ssl_wrapper",
    "//python_tools/plotting:plot_pass_generator",
    "//proto:tbots_py_proto",
    "//proto:software_py_proto",
]

PYTHON_BINDINGS = [
    "//software/python_bindings:pass_generator.so",
    "//software/python_bindings:passing.so",
    "//software/python_bindings:world.so",
    "//software/python_bindings:keep_away.so",
]

py_test(
    name = "test_hermetic_python_environment",
    srcs = ["test_hermetic_python_environment.py"],
)

py_library(
    name = "proto_log",
    srcs = ["proto_log.py"],
    deps = [
        "//proto:software_py_proto",
        requirement("protobuf"),
    ],
)

py_library(
    name = "sensor_proto_log",
    srcs = ["sensor_proto_log.py"],
    deps = [
        ":proto_log",
        "//proto:software_py_proto",
    ],
)

py_binary(
    name = "jupyter",
    srcs = ["jupyter.py"],
    # glob all the notebooks in this directory
    data = [
        ":notebooks",
    ] + PYTHON_BINDINGS,
    deps = NOTEBOOK_DEPS,
)

py_test(
    name = "notebooks_test",
    srcs = ["notebooks_test.py"],
    data = [
        ":notebooks",
    ] + PYTHON_BINDINGS,
    deps = NOTEBOOK_DEPS,
)
