package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_binary(
    name = "server",
    srcs = ["server.py"],
    deps = [
        requirement("tftpy"),
    ],
)

py_binary(
    name = "client",
    srcs = ["client.py"],
    deps = [
        requirement("tftpy"),
        "//python_tools:robot_broadcast_receiver",
    ],
)
