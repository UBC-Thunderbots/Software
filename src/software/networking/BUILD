package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tbots_network_exception",
    hdrs = ["tbots_network_exception.h"],
    srcs = ["tbots_network_exception.cpp"],
)

py_library(
    name = "ssl_proto_communication",
    srcs = ["ssl_proto_communication.py"],
    deps = [
        "//proto/message_translation:py_tbots_protobuf",
    ],
)
