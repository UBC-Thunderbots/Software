package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ssl_gamecontroller_client",
    srcs = ["ssl_gamecontroller_client.cpp"],
    hdrs = ["ssl_gamecontroller_client.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//software/proto:ssl_cc_proto",
        "@boost//:asio",
        "@g3log",
    ],
)

cc_library(
    name = "ssl_vision_client",
    srcs = ["ssl_vision_client.cpp"],
    hdrs = ["ssl_vision_client.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//software/proto:ssl_cc_proto",
        "@boost//:asio",
        "@g3log",
    ],
)

cc_library(
    name = "ssl_protobuf_reader",
    srcs = ["ssl_protobuf_reader.cpp"],
    hdrs = ["ssl_protobuf_reader.h"],
    deps = [
        "//software/parameter:dynamic_parameters",
        "//software/proto:ssl_cc_proto",
        "//software/sensor_fusion:refbox_data",
        "//software/sensor_fusion:vision_detection",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:team",
        "@g3log",
    ],
)

cc_library(
    name = "network_client",
    srcs = [
        "network_client.cpp",
    ],
    hdrs = ["network_client.h"],
    deps = [
        ":ssl_gamecontroller_client",
        ":ssl_protobuf_reader",
        ":ssl_vision_client",
        "//software/backend/input/network/filter:ball_filter",
        "//software/backend/input/network/filter:robot_filter",
        "//software/backend/input/network/filter:robot_team_filter",
        "//software/proto:ssl_cc_proto",
        "//software/world",
        "@g3log",
    ],
)
