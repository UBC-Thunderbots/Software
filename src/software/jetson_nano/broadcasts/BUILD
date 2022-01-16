package(default_visibility = ["//visibility:public"])

load("@jetson_nano_deps//:requirements.bzl", "requirement")

py_binary(
    name = "robot_broadcast_receiver",
    srcs = ["robot_broadcast_receiver.py"],
    deps = ["//proto:announcement_py_proto"],
)

py_binary(
    name = "robot_broadcast_sender",
    srcs = ["robot_broadcast_sender.py"],
    deps = ["//proto:announcement_py_proto"],
)
