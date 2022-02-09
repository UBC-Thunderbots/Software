package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    srcs = ["backend.cpp"],
    hdrs = ["backend.h"],
    deps = [
        "//proto:sensor_msg_cc_proto",
        "//proto/message_translation:tbots_protobuf",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/world",
    ],
)

cc_library(
    name = "wifi_backend",
    srcs = ["wifi_backend.cpp"],
    hdrs = ["wifi_backend.h"],
    deps = [
        ":backend",
        ":ssl_proto_client",
        "//proto:defending_side_msg_cc_proto",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:defending_side",
        "//proto/message_translation:tbots_protobuf",
        "//proto/primitive:primitive_msg_factory",
        "//shared:constants",
        "//software:constants",
        "//software/estop:boost_uart_communication",
        "//software/estop:threaded_estop_reader",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/util/generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "simulator_backend",
    srcs = ["simulator_backend.cpp"],
    hdrs = ["simulator_backend.h"],
    deps = [
        ":backend",
        ":ssl_proto_client",
        "//proto:defending_side_msg_cc_proto",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:defending_side",
        "//proto/message_translation:ssl_wrapper",
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software:constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/networking:threaded_unix_sender",
        "//software/util/generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "replay_backend",
    srcs = ["replay_backend.cpp"],
    hdrs = ["replay_backend.h"],
    deps = [
        ":backend",
        ":ssl_proto_client",
        "//proto/logging:proto_log_reader",
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software:constants",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/util/generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "all_backends",
    deps = [
        ":replay_backend",
        ":simulator_backend",
        ":wifi_backend",
    ],
)

cc_library(
    name = "ssl_proto_client",
    srcs = ["ssl_proto_client.cpp"],
    hdrs = ["ssl_proto_client.h"],
    deps = [
        "//proto:ssl_cc_proto",
        "//shared/parameter:cpp_configs",
        "//software:constants",
        "//software/networking:threaded_proto_udp_listener",
    ],
)
