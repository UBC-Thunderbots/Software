cc_library(
    name = "annunciator",
    srcs = ["annunciator.cpp"],
    hdrs = ["annunciator.h"],
    deps = [
        ":messages",
        "//software/backend:robot_status",
        "//software/util:constants",
    ],
)

cc_library(
    name = "send_reliable_message_operation",
    srcs = ["send_reliable_message_operation.cpp"],
    hdrs = ["send_reliable_message_operation.h"],
    deps = [
        ":messages",
        "//software/backend/output/radio/mrf/usb:libusb",
    ],
)

cc_library(
    name = "messages",
    hdrs = ["messages.h"],
    deps = [],
)

cc_library(
    name = "dongle",
    srcs = ["dongle.cpp"],
    hdrs = ["dongle.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":annunciator",
        ":send_reliable_message_operation",
        "//software/ai/primitive",
        "//software/geom:all",
    ],
)
