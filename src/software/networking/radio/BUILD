package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_radio_listener",
    hdrs = [
        "proto_radio_listener.hpp",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "//software/logger",
        "//software/util/typename",
    ],
)

cc_library(
    name = "radio_sender",
    srcs = [
        "radio_sender.cpp",
    ],
    hdrs = [
        "radio_sender.h",
    ],
    visibility = ["//visibility:private"],
    deps = [],
)

cc_library(
    name = "threaded_proto_radio_listener",
    hdrs = [
        "threaded_proto_radio_listener.hpp",
    ],
    deps = [
        ":proto_radio_listener",
    ],
)

cc_library(
    name = "threaded_proto_radio_sender",
    hdrs = [
        "threaded_proto_radio_sender.hpp",
    ],
    deps = [
        ":threaded_radio_sender",
    ],
)

cc_library(
    name = "threaded_radio_sender",
    srcs = [
        "threaded_radio_sender.cpp",
    ],
    hdrs = [
        "threaded_radio_sender.h",
    ],
    deps = [
        ":radio_sender",
    ],
)
