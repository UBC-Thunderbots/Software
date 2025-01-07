package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tbots_network_exception",
    srcs = ["tbots_network_exception.cpp"],
    hdrs = ["tbots_network_exception.h"],
)

py_library(
    name = "ssl_proto_communication",
    srcs = ["ssl_proto_communication.py"],
    deps = [
        "//proto/message_translation:py_tbots_protobuf",
    ],
)
