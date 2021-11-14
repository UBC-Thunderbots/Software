package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_binary(
    name = "unix_receiver",
    srcs = ["unix_receiver.py"],
    deps = [
        "//proto:software_py_proto",
    ],
)

py_binary(
    name = "unix_sender",
    srcs = ["unix_sender.py"],
    deps = [
        "//proto:software_py_proto",
    ],
)
