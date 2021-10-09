package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_binary(
    name = "receiver",
    srcs = ["receiver.py"],
    deps = [
        requirement("tftpy"),
    ],
)

py_binary(
    name = "uploader",
    srcs = ["uploader.py"],
    deps = [
        requirement("tftpy"),
        "//python_tools:robot_broadcast_receiver",
    ],
)
