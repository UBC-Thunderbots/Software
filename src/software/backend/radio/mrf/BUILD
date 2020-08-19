package(default_visibility = ["//software/backend/radio:__subpackages__"])

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
        "//software/backend/radio:robot_status",
        "//software/backend/radio/mrf/usb:libusb",
        "//software/primitive",
    ],
)
