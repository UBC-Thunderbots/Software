package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_udp_client",
    hdrs = [
        "proto_udp_client.hpp",
    ],
    deps = [
        "//software/logger",
        "//software/util/typename",
    ],
)

cc_library(
    name = "proto_udp_listener",
    hdrs = [
        "proto_udp_listener.hpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "//software/util/typename",
    ],
)

cc_library(
    name = "udp_sender",
    srcs = [
        "udp_sender.cpp",
    ],
    hdrs = [
        "udp_sender.h",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_udp_listener",
    hdrs = [
        "threaded_proto_udp_listener.hpp",
    ],
    deps = [
        ":proto_udp_listener",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_udp_sender",
    hdrs = [
        "threaded_proto_udp_sender.hpp",
    ],
    deps = [
        ":udp_sender",
        "@boost//:asio",
    ],
)

cc_library(
    name = "unix_sender",
    srcs = [
        "unix_sender.cpp",
    ],
    hdrs = [
        "unix_sender.h",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software:constants",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_udp_sender",
    srcs = [
        "threaded_udp_sender.cpp",
    ],
    hdrs = [
        "threaded_udp_sender.h",
    ],
    deps = [
        ":udp_sender",
    ],
)

cc_library(
    name = "proto_unix_listener",
    hdrs = [
        "proto_unix_listener.hpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software:constants",
        "//software/util/typename",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_unix_sender",
    srcs = [
        "threaded_unix_sender.cpp",
    ],
    hdrs = [
        "threaded_unix_sender.h",
    ],
    deps = [
        ":unix_sender",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_unix_sender",
    hdrs = [
        "threaded_proto_unix_sender.hpp",
    ],
    deps = [
        ":unix_sender",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_unix_listener",
    hdrs = [
        "threaded_proto_unix_listener.hpp",
    ],
    deps = [
        ":proto_unix_listener",
        "@boost//:asio",
    ],
)

py_library(
    name = "threaded_unix_listener_py",
    srcs = ["threaded_unix_listener.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//proto:software_py_proto",
        "//software/logger:py_logger",
    ],
)

py_library(
    name = "threaded_unix_sender_py",
    srcs = ["threaded_unix_sender.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//proto:software_py_proto",
        "//software/logger:py_logger",
    ],
)
