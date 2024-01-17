package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "simulator",
    srcs = [
        "simulator.py",
        "util.py",
    ],
    data = [
        "//software:er_force_simulator_main",
    ],
)

py_library(
    name = "game_controller",
    srcs = [
        "game_controller.py",
        "util.py",
    ],
    deps = [
        "//proto:import_all_protos",
        "//software/networking:ssl_proto_communication",
    ],
)

py_library(
    name = "full_system",
    srcs = [
        "full_system.py",
        "util.py",
    ],
    data = [
        "//software:unix_full_system",
    ],
)

py_library(
    name = "tigers_autoref",
    srcs = [
        "tigers_autoref.py",
    ],
    data = [
        "//software/autoref:run_autoref",
    ],
    deps = [
        ":game_controller",
        "//software/thunderscope:time_provider",
    ],
)
