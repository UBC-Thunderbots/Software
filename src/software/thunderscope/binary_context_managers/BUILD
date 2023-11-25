package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "simulator",
    srcs = [
        "util.py",
        "simulator.py",
    ],
    data = [
        "//software:er_force_simulator_main",
    ],
)

py_library(
    name = "game_controller",
    srcs = [
        "util.py",
        "game_controller.py",
    ],
)

py_library(
    name = "full_system",
    srcs = [
        "util.py",
        "full_system.py",
    ],
    data = [
        "//software:unix_full_system",
    ],
)
