package(default_visibility = ["//visibility:public"])

load("@thunderscope_deps//:requirements.bzl", "requirement")

py_library(
    name = "field",
    srcs = ["field.py"],
    deps = [
        ":field_layer",
        "//software/thunderscope/replay:proto_player",
        "//software/thunderscope/replay:replay_controls",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "field_layer",
    srcs = ["field_layer.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "obstacle_layer",
    srcs = ["obstacle_layer.py"],
    deps = [
        ":field_layer",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "hrvo_layer",
    srcs = ["hrvo_layer.py"],
    deps = [
        ":field_layer",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "path_layer",
    srcs = ["path_layer.py"],
    deps = [
        ":field_layer",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "passing_layer",
    srcs = ["passing_layer.py"],
    deps = [
        ":field_layer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "world_layer",
    srcs = ["world_layer.py"],
    deps = [
        ":field_layer",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "sandbox_world_layer",
    srcs = ["sandbox_world_layer.py"],
    deps = [
        ":world_layer",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "sandbox_controls",
    srcs = ["sandbox_controls.py"],
    deps = [
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "simulator_layer",
    srcs = ["simulator_layer.py"],
    deps = [
        ":field_layer",
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)

py_library(
    name = "validation_layer",
    srcs = ["validation_layer.py"],
    deps = [
        ":field_layer",
        "//proto:import_all_protos",
        "//software/thunderscope:thread_safe_buffer",
        requirement("pyqtgraph"),
    ],
)
