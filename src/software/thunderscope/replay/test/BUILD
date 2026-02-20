load("@simulated_tests_deps//:requirements.bzl", "requirement")
load("@aspect_rules_py//py:defs.bzl", "py_binary", "py_library", "py_test")
package(default_visibility = ["//visibility:public"])

py_test(
    name = "replay_corruption_test",
    srcs = [
        "replay_corruption_test.py",
    ],
    env = {
        "LD_LIBRARY_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/lib",
        "QTWEBENGINEPROCESS_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/libexec/QtWebEngineProcess",
    },
    package_collisions = "warning",
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
        requirement("pyqt6"),
        requirement("numpy"),
        requirement("pyqtdarktheme-fork"),
        requirement("pyqtgraph"),
    ],
)

py_test(
    name = "replay_indexing_test",
    srcs = [
        "replay_indexing_test.py",
    ],
    env = {
        "LD_LIBRARY_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/lib",
        "QTWEBENGINEPROCESS_PATH": "../.thunderscope_main.venv/lib/python3.12/site-packages/PyQt6/Qt6/libexec/QtWebEngineProcess",
    },
    package_collisions = "warning",
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
        requirement("pyqt6"),
        requirement("numpy"),
        requirement("pyqtdarktheme-fork"),
        requirement("pyqtgraph"),
    ],
)
