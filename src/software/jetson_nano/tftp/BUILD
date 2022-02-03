package(default_visibility = ["//visibility:public"])

load("@jetson_nano_deps//:requirements.bzl", "requirement")

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
        "//software/jetson_nano/broadcasts:robot_broadcast_receiver",
    ],
)
