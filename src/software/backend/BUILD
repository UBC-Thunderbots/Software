package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    srcs = ["backend.cpp"],
    hdrs = ["backend.h"],
    deps = [
        ":robot_status",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/primitive",
        "//software/proto:sensor_msg_cc_proto",
        "//software/world",
    ],
)

cc_library(
    name = "radio_backend",
    srcs = ["radio_backend.cpp"],
    hdrs = ["radio_backend.h"],
    deps = [
        ":backend",
        ":ssl_proto_client",
        "//software/backend/input/network/networking:network_client",
        "//software/backend/output/radio:radio_output",
        "//software/util/design_patterns:generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "wifi_backend",
    srcs = ["wifi_backend.cpp"],
    hdrs = ["wifi_backend.h"],
    deps = [
        ":backend",
        ":ssl_proto_client",
        "//shared:constants",
        "//software:constants",
        "//software/backend/input/network/networking:network_client",
        "//software/networking:threaded_proto_multicast_listener",
        "//software/networking:threaded_proto_multicast_sender",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/util/design_patterns:generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_test(
    name = "robot_status_test",
    srcs = ["robot_status_test.cpp"],
    deps = [
        ":robot_status",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "all_backends",
    deps = [
        ":radio_backend",
        ":wifi_backend",
    ],
)

cc_library(
    name = "ssl_proto_client",
    srcs = ["ssl_proto_client.cpp"],
    hdrs = ["ssl_proto_client.h"],
    deps = [
        "//software:constants",
        "//software/networking:threaded_proto_multicast_listener",
        "//software/parameter:dynamic_parameters",
        "//software/proto:ssl_cc_proto",
    ],
)

cc_library(
    name = "robot_status",
    srcs = ["robot_status.cpp"],
    hdrs = ["robot_status.h"],
    deps = [
        "//shared/proto:tbots_cc_proto",
        "//software/logger",
    ],
)
