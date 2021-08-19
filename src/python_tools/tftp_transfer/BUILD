package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_binary(
    name = "host",
    srcs = ["host.py"],
    deps = [
        requirement("tftpy"),
    ],
)

py_binary(
    name = "client",
    srcs = ["client.py"],
    deps = [
        requirement("tftpy"),
    ],
)
