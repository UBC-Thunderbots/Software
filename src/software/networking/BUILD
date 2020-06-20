package(default_visibility = ["//visibility:public"])

cc_library(
    name = "threaded_proto_multicast_listener",
    # We include the non-threaded class as srcs so the headers
    # cannot accidentally be seen or included by other code
    srcs = [
        "proto_multicast_listener.h",
        "proto_multicast_listener.hpp",
    ],
    hdrs = [
        "threaded_proto_multicast_listener.h",
        "threaded_proto_multicast_listener.tpp",
    ],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)

cc_library(
    name = "threaded_proto_multicast_sender",
    # We include the non-threaded class as srcs so the headers
    # cannot accidentally be seen or included by other code
    srcs = [
        "proto_multicast_sender.h",
        "proto_multicast_sender.hpp",
    ],
    hdrs = [
        "threaded_proto_multicast_sender.h",
        "threaded_proto_multicast_sender.tpp",
    ],
    deps = [
        "//software/logger",
        "@boost//:asio",
    ],
)
