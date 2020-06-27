package(default_visibility = ["//visibility:private"])

cc_library(
    name = "messages",
    hdrs = ["messages.h"],
    deps = [],
)

cc_library(
    name = "dongle",
    srcs = [
        "annunciator.cpp",
        "dongle.cpp",
        "send_reliable_message_operation.cpp",
    ],
    hdrs = [
        "annunciator.h",
        "dongle.h",
        "send_reliable_message_operation.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":messages",
        "//shared:constants",
        "//software:constants",
        "//software/backend:robot_status",
        "//software/backend/output/radio/mrf/usb:libusb",
        "//software/geom",
        "//software/primitive",
        "//software/proto/message_translation:proto_creator_primitive_visitor",
    ],
)
