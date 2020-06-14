package(default_visibility = ["//visibility:public"])

cc_library(
    name = "network_filter",
    srcs = ["network_filter.cpp"],
    hdrs = ["network_filter.h"],
    deps = [
        "//software/backend/input/network/filter:ball_filter",
        "//software/backend/input/network/filter:robot_filter",
        "//software/backend/input/network/filter:robot_team_filter",
        "//software/parameter:dynamic_parameters",
        "//software/proto:ssl_cc_proto",
        "//software/sensor_fusion:refbox_data",
        "//software/world:ball",
        "//software/world:field",
        "//software/world:team",
        "//software/world:timestamped_ball_state",
    ],
)

cc_library(
    name = "network_client",
    srcs = [
        "network_client.cpp",
        # TODO: We shouldn't have to have the *_client.cpp files here, but full_system
        #       fails to link if we don't.......
        #        "ssl_vision_client.cpp",
        #        "ssl_gamecontroller_client.cpp",
    ],
    hdrs = ["network_client.h"],
    deps = [
        ":network_filter",
        "//software/backend/input/network/filter:ball_filter",
        "//software/backend/input/network/filter:robot_filter",
        "//software/backend/input/network/filter:robot_team_filter",
        "//software/logger",
        "//software/networking:proto_multicast_listener",
        "//software/parameter:dynamic_parameters",
        "//software/proto:ssl_cc_proto",
        "//software/world",
    ],
)
