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
