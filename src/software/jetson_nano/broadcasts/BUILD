package(default_visibility = ["//visibility:public"])

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

genrule(
    name = "robot_broadcast_sender_zip",
    outs = ["robot_broadcast_sender_zip.zip"],
    cmd = "zip -r $@ $(location :robot_broadcast_sender)*",
    tools = [":robot_broadcast_sender"],
)
