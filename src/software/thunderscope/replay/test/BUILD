package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_test(
    name = "replay_corruption_test",
    srcs = [
        "replay_corruption_test.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//software:conftest",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope/replay:proto_player",
        requirement("pytest"),
    ],
)

py_test(
    name = "replay_indexing_test",
    srcs = [
        "replay_indexing_test.py",
    ],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:erforce_py_proto",
        "//proto:import_all_protos",
        "//software:conftest",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope/replay:proto_player",
        "//software/thunderscope/replay/test:replay_corruption_test",
        requirement("pytest"),
    ],
)
