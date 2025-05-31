load("@nanopb//extra/bazel:nanopb_cc_proto_library.bzl", "cc_nanopb_proto_library")
load("@protobuf//bazel:proto_library.bzl", "proto_library")
#load("//proto:nanopb.bzl", "nanopb_proto_library")

load("@protobuf//bazel:py_proto_library.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "geneva_proto",
    srcs = [
        "geneva_slot.proto",
    ],
)

proto_library(
    name = "robot_proto",
    srcs = [
        "geometry.proto",
        "robot_crash_msg.proto",
        "robot_log_msg.proto",
        "robot_status_msg.proto",
        "tbots_timestamp_msg.proto",
    ],
    deps = [
        "geneva_proto",
        "@nanopb//:nanopb_proto",
    ],
)

proto_library(
    name = "tbots_proto",
    srcs = [
        "ball.proto",
        "game_state.proto",
        "ip_notification.proto",
        "parameters.proto",
        "play.proto",
        "power_frame_msg.proto",
        "primitive.proto",
        "replay_bookmark.proto",
        "robot_statistic.proto",
        "tactic.proto",
        "tbots_software_msgs.proto",
        "team.proto",
        "vision.proto",
        "world.proto",
    ],
    deps = [
        "geneva_proto",
        "robot_proto",
        "@nanopb//:nanopb_proto",
        "@protobuf//:descriptor_proto",
    ],
)

cc_proto_library(
    name = "tbots_cc_proto",
    deps = [":tbots_proto"],
)

cc_nanopb_proto_library(
    name = "tbots_nanopb_proto",
    protos = [":tbots_proto"],
)

py_proto_library(
    name = "tbots_py_proto",
    deps = [
        ":tbots_proto",
    ],
)

proto_library(
    name = "ssl_proto",
    srcs = [
        "ssl_autoref_ci.proto",
        "ssl_gc_api.proto",
        "ssl_gc_change.proto",
        "ssl_gc_ci.proto",
        "ssl_gc_common.proto",
        "ssl_gc_engine.proto",
        "ssl_gc_engine_config.proto",
        "ssl_gc_game_event.proto",
        "ssl_gc_geometry.proto",
        "ssl_gc_referee_message.proto",
        "ssl_gc_state.proto",
        "ssl_vision_detection.proto",
        "ssl_vision_detection_tracked.proto",
        "ssl_vision_geometry.proto",
        "ssl_vision_wrapper.proto",
        "ssl_vision_wrapper_tracked.proto",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@protobuf//:duration_proto",
        "@protobuf//:timestamp_proto",
        "@protobuf//:wrappers_proto",
    ],
)

# These proto files are based on the following commit:
# https://github.com/RoboCup-SSL/ssl-simulation-protocol/commit/b455b4b8a1ed6f6aebaf8e8cd3721b7e130aa9ce
proto_library(
    name = "ssl_simulation_proto",
    srcs = [
        "ssl_simulation_config.proto",
        "ssl_simulation_control.proto",
        "ssl_simulation_error.proto",
        "ssl_simulation_robot_control.proto",
        "ssl_simulation_robot_feedback.proto",
        "ssl_simulation_synchronous.proto",
    ],
    visibility = ["//visibility:private"],
    deps = [
        ":ssl_proto",
        "@protobuf//:any_proto",
    ],
)

cc_proto_library(
    name = "ssl_simulation_cc_proto",
    deps = [":ssl_simulation_proto"],
)

cc_proto_library(
    name = "any_cc_proto",
    deps = ["@protobuf//:any_proto"],
)

proto_library(
    name = "sensor_msg_proto",
    srcs = [
        "sensor_msg.proto",
    ],
    visibility = ["//visibility:private"],
    deps = [
        "robot_proto",
        ":ssl_proto",
        ":tbots_proto",
    ],
)

proto_library(
    name = "validation_proto",
    srcs = [
        "validation.proto",
    ],
    deps = [
        ":tbots_proto",
    ],
)

proto_library(
    name = "visualization_proto",
    srcs = [
        "visualization.proto",
    ],
    deps = [
        ":tbots_proto",
        "@protobuf//:any_proto",
    ],
)

proto_library(
    name = "play_info_msg_proto",
    srcs = [
        "play_info_msg.proto",
    ],
    visibility = ["//visibility:private"],
)

cc_proto_library(
    name = "ssl_cc_proto",
    deps = [":ssl_proto"],
)

cc_proto_library(
    name = "sensor_msg_cc_proto",
    deps = [":sensor_msg_proto"],
)

cc_proto_library(
    name = "validation_cc_proto",
    deps = [":validation_proto"],
)

cc_proto_library(
    name = "visualization_cc_proto",
    deps = [":visualization_proto"],
)

cc_proto_library(
    name = "play_info_msg_cc_proto",
    deps = [":play_info_msg_proto"],
)

proto_library(
    name = "software_proto",
    srcs = [
        "manual_gc_command.proto",
        "play_info_msg.proto",
        "sensor_msg.proto",
        "ssl_autoref_ci.proto",
        "ssl_gc_api.proto",
        "ssl_gc_change.proto",
        "ssl_gc_ci.proto",
        "ssl_gc_common.proto",
        "ssl_gc_engine.proto",
        "ssl_gc_engine_config.proto",
        "ssl_gc_game_event.proto",
        "ssl_gc_geometry.proto",
        "ssl_gc_referee_message.proto",
        "ssl_gc_state.proto",
        "ssl_vision_detection.proto",
        "ssl_vision_detection_tracked.proto",
        "ssl_vision_geometry.proto",
        "ssl_vision_wrapper.proto",
        "ssl_vision_wrapper_tracked.proto",
    ],
    deps = [
        ":tbots_proto",
        "@protobuf//:duration_proto",
        "@protobuf//:timestamp_proto",
        "@protobuf//:wrappers_proto",
    ],
)

py_proto_library(
    name = "software_py_proto",
    deps = [
        ":software_proto",
    ],
)

py_library(
    name = "import_all_protos",
    srcs = [
        "import_all_protos.py",
    ],
    deps = [
        ":software_py_proto",
        ":tbots_py_proto",
    ],
)
