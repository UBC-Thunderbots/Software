load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator_team_interface",
    hdrs = ["simulator_team_interface.h"],
    deps = [
        "//software/simulation:er_force_simulator",
    ],
)

cc_library(
    name = "unix_primitive_interface",
    hdrs = ["unix_primitive_interface.h"],
    deps = [
        ":simulator_team_interface",
        "//proto:ssl_cc_proto",
        "//proto:tbots_cc_proto",
        "//software/networking/unix:threaded_proto_unix_listener",
        "//software/networking/unix:threaded_proto_unix_sender",
    ],
)

cc_library(
    name = "udp_ssl_interface",
    hdrs = ["udp_ssl_interface.h"],
    deps = [
        ":simulator_team_interface",
        "//proto:ssl_simulation_cc_proto",
        "//software/logger",
        "//software/networking/udp:threaded_proto_udp_bidirectional",
    ],
)
