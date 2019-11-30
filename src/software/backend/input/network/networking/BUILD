package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ssl_gamecontroller_client",
    srcs = ["ssl_gamecontroller_client.cpp"],
    hdrs = ["ssl_gamecontroller_client.h"],
    #    visibility = ["//visibility:private"],
    deps = [
        "//software/proto:ssl_cc_proto",
        "@g3log",
        "@boost//:asio",
    ],
)

cc_library(
    name = "ssl_vision_client",
    srcs = ["ssl_vision_client.cpp"],
    hdrs = ["ssl_vision_client.h"],
    #    visibility = ["//visibility:private"],
    deps = [
        "//software/proto:ssl_cc_proto",
        "@g3log",
        "@boost//:asio",
    ],
)

cc_library(
    name = "network_filter",
    srcs = ["network_filter.cpp"],
    hdrs = ["network_filter.h"],
    deps = [
        "//software/world:ball",
        "//software/world:ball_state",
        "//software/world:field",
        "//software/sensor_fusion:refbox_data",
        "//software/world:team",
        "//software/backend/input/network/filter:ball_filter",
        "//software/backend/input/network/filter:robot_filter",
        "//software/backend/input/network/filter:robot_team_filter",
        "//software/proto:ssl_cc_proto",
        "//software/util/parameter:dynamic_parameters",
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
        ":ssl_gamecontroller_client",
        ":ssl_vision_client",
        "//software/world",
        "//software/backend/input/network/filter:ball_filter",
        "//software/backend/input/network/filter:robot_filter",
        "//software/backend/input/network/filter:robot_team_filter",
        "//software/proto:ssl_cc_proto",
        "@g3log",
    ],
)
