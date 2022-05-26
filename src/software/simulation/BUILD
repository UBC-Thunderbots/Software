package(default_visibility = ["//visibility:public"])

cc_library(
    name = "er_force_simulator",
    srcs = ["er_force_simulator.cpp"],
    hdrs = ["er_force_simulator.h"],
    data = [
        "//extlibs/er_force_sim/config/simulator:2020.txt",
        "//extlibs/er_force_sim/config/simulator:2020B.txt",
    ],
    deps = [
        "//extlibs/er_force_sim/src/amun/simulator:simulator_qt",
        "//proto:defending_side_msg_cc_proto",
        "//proto/message_translation:ssl_detection",
        "//proto/message_translation:ssl_geometry",
        "//proto/message_translation:ssl_simulation_robot_control",
        "//proto/message_translation:ssl_wrapper",
        "//software/jetson_nano:primitive_executor",
        "//software/world",
        "//software/world:field",
        "//software/world:team_colour",
    ],
)

cc_test(
    name = "er_force_simulator_test",
    srcs = ["er_force_simulator_test.cpp"],
    deps = [
        ":er_force_simulator",
        "//proto/message_translation:er_force_world",
        "//proto/primitive:primitive_msg_factory",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
        "//software/world",
    ],
)
