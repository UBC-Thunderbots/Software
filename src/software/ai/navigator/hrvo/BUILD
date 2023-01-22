package(default_visibility = ["//visibility:public"])
cc_library(
    name="hrvo",
    srcs=[
        "hrvo_simulator.cpp",
        "HRVOAgent.cpp",
        "lv_agent.cpp",
        "path_point.cpp",
        "path.cpp"
    ],
    hdrs=[
        "hrvo_simulator.h",
        "HRVOAgent.h",
        "lv_agent.h",
        "path_point.h",
        "path.h"
    ],
    deps=[
        "//proto:visualization_cc_proto",
        "//proto/primitive:primitive_msg_factory",
        "//software/ai/navigator/obstacle:robot_navigation_obstacle_factory",
        "//software/ai/navigator/path_planner/hrvo:velocity_obstacle",
        "//software/geom:vector",
        "//software/networking:threaded_proto_unix_sender",
        "//software/world",
    ]
)

