package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "proto_player",
    srcs = [
        "proto_player.py",
    ],
    data = [
        "//software:py_constants.so",
        "//software:python_bindings.so",
    ],
    deps = [
        requirement("pyqtgraph"),
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//software/thunderscope:constants",
        "//software/thunderscope:proto_unix_io",
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
        "replay_file_debugging_script.py",
    ],
    deps = [
        "//proto:import_all_protos",
        ":proto_player",
    ],
)
