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
        "//software/simulated_tests:simulated_test_fixture",
        "//software/thunderscope:proto_unix_io",
        "//software/thunderscope/replay:proto_player",
        requirement("pytest"),
    ],
)
