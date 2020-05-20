load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "ssl_proto",
    srcs = [
        "messages_robocup_ssl_detection.proto",
        "messages_robocup_ssl_geometry.proto",
        "messages_robocup_ssl_wrapper.proto",
        "ssl_game_controller_common.proto",
        "ssl_game_event.proto",
        "ssl_game_event_2019.proto",
        "ssl_referee.proto",
    ],
    visibility = ["//visibility:private"],
)

proto_library(
    name = "grsim_proto",
    srcs = [
        "grSim_Commands.proto",
        "grSim_Packet.proto",
        "grSim_Replacement.proto",
    ],
    visibility = ["//visibility:private"],
    deps = [],
)

proto_library(
    name = "sensor_msg_proto",
    srcs = [
        "sensor_msg.proto",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":ssl_proto",
        "//shared/proto:tbots_proto",
    ],
)

cc_proto_library(
    name = "ssl_cc_proto",
    deps = [":ssl_proto"],
)

cc_proto_library(
    name = "grsim_cc_proto",
    deps = [":grsim_proto"],
)

cc_proto_library(
    name = "sensor_msg_cc_proto",
    deps = [":sensor_msg_proto"],
)
