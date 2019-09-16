cc_library(
    name = "mrf_primitive_visitor",
    srcs = ["mrf_primitive_visitor.cpp"],
    hdrs = ["mrf_primitive_visitor.h"],
    deps = [
        "//shared:constants",
        "//shared:firmware_primitive_type",
        "//software/ai/primitive:all_primitives",
        "//software/ai/primitive:primitive_visitor",
    ],
)

cc_test(
    name = "mrf_primitive_visitor_test",
    srcs = ["mrf_primitive_visitor_test.cpp"],
    deps = [
        ":mrf_primitive_visitor",
        "@gtest//:gtest_main",
    ],
)

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
        ":mrf_primitive_visitor",
        "//shared:constants",
        "//software/ai/primitive",
        "//software/backend:robot_status",
        "//software/backend/output/radio/mrf/usb:libusb",
        "//software/geom:all",
        "//software/util:constants",
    ],
)
