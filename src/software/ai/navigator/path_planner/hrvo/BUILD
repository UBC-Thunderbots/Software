package(default_visibility = ["//visibility:public"])

cc_library(
    name = "velocity_obstacle",
    srcs = ["velocity_obstacle.cpp"],
    hdrs = [
        "velocity_obstacle.h",
    ],
    deps = [
        "//software/geom:circle",
        "//software/geom:vector",
        "//software/logger",
    ],
)

cc_library(
    name="hrvo",
    srcs=[
        "hrvo_simulator.cpp",
        "hrvo_agent.cpp",
        "lv_agent.cpp",
        "path_point.cpp",
        "robot_path.cpp"
    ],
    hdrs=[
        "hrvo_simulator.h",
        "hrvo_agent.h",
        "lv_agent.h",
        "path_point.h",
        "robot_path.h"
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

cc_test(
    name = "velocity_obstacle_test",
    srcs = ["velocity_obstacle_test.cpp"],
    deps = [
        "velocity_obstacle",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
