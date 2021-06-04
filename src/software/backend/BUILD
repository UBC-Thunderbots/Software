package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    srcs = ["backend.cpp"],
    hdrs = ["backend.h"],
    deps = [
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/proto:sensor_msg_cc_proto",
        "//software/proto/message_translation:tbots_protobuf",
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
        "//software/backend/radio:radio_output",
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
        "//shared/proto:tbots_cc_proto",
        "//software:constants",
        "//software/estop:boost_uart_communication",
        "//software/estop:threaded_estop_reader",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/proto:defending_side_msg_cc_proto",
        "//software/proto/message_translation:defending_side",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/proto/primitive:primitive_msg_factory",
        "//software/util/design_patterns:generic_factory",
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
        "//shared:constants",
        "//shared/proto:tbots_cc_proto",
        "//software:constants",
        "//software/logger",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/proto:defending_side_msg_cc_proto",
        "//software/proto/message_translation:defending_side",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/util/design_patterns:generic_factory",
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
        "//shared:constants",
        "//software:constants",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/proto/logging:proto_log_reader",
        "//software/proto/message_translation:tbots_protobuf",
        "//software/util/design_patterns:generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "all_backends",
    deps = [
        ":radio_backend",
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
        "//shared/parameter:cpp_configs",
        "//software:constants",
        "//software/networking:threaded_proto_udp_listener",
        "//software/proto:ssl_cc_proto",
    ],
)
