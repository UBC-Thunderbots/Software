package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_test(
    name = "test_hermetic_python_environment",
    srcs = ["test_hermetic_python_environment.py"],
)

py_library(
    name = "proto_log",
    srcs = ["proto_log.py"],
    deps = [
        "//software/proto:software_py_proto",
        requirement("protobuf"),
    ],
)

py_library(
    name = "sensor_proto_log",
    srcs = ["sensor_proto_log.py"],
    deps = [
        ":proto_log",
        "//software/proto:software_py_proto",
    ],
)

py_binary(
    name = "jupyter",
    srcs = ["jupyter.py"],
    # glob all the notebooks in this directory
    data = [":notebooks"],
    deps = [
        requirement("jupyter"),
        requirement("notebook"),
        requirement("numpy"),
        requirement("matplotlib"),
        ":proto_log",
        ":sensor_proto_log",
        "//shared/proto:tbots_py_proto",
        "//software/proto:software_py_proto",
    ],
)
