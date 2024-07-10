package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "proto_logger",
    srcs = [
        "proto_logger.py",
        "replay_constants.py",
    ],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "proto_player",
    srcs = [
        "proto_player.py",
        "replay_constants.py",
    ],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "replay_controls",
    srcs = [
        "replay_controls.py",
    ],
    deps = [
        requirement("pyqtgraph"),
        ":proto_player",
    ],
)

py_binary(
    name = "replay_file_debugging_script",
    srcs = [
        "replay_constants.py",
        "replay_file_debugging_script.py",
    ],
    deps = [
        "//proto:import_all_protos",
    ],
)
