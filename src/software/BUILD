package(default_visibility = ["//visibility:public"])

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")

cc_binary(
    name = "unix_full_system",
    srcs = ["unix_full_system_main.cpp"],
    deps = [
        "//extlibs/er_force_sim/src/protobuf:protobuf_cc_lib",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:ssl_wrapper",
        "//software:constants",
        "//software/ai:threaded_ai",
        "//software/backend",
        "//software/backend:all_backends",
        "//software/estop:arduino_util",
        "//software/logger",
        "//software/multithreading:observer_subject_adapter",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/networking:threaded_proto_unix_listener",
        "//software/sensor_fusion:threaded_sensor_fusion",
        "//software/util/generic_factory",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "er_force_simulator_main",
    srcs = ["er_force_simulator_main.cpp"],
    deps = [
        "//software:constants",
        "//software/networking:threaded_proto_unix_listener",
        "//software/networking:threaded_proto_unix_sender",
        "//software/simulation:er_force_simulator",
        "@boost//:program_options",
    ],
)

cc_binary(
    name = "network_log_listener_main",
    srcs = ["network_log_listener_main.cpp"],
    deps = [
        "//proto:tbots_cc_proto",
        "//shared:constants",
        "//software/networking:threaded_proto_udp_listener",
        "@boost//:program_options",
    ],
)

cc_library(
    name = "constants",
    hdrs = ["constants.h"],
    deps = [
        "//shared:constants",
    ],
)

pybind_library(
    name = "py_constants_lib",
    srcs = ["py_constants.cpp"],
    deps = [
        ":constants",
        "//shared:constants",
    ],
)

pybind_extension(
    name = "py_constants",
    deps = [
        ":py_constants_lib",
    ],
)

pybind_library(
    name = "python_bindings_lib",
    srcs = ["python_bindings.cpp"],
    deps = [
        "//proto:ssl_cc_proto",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:tbots_geometry",
        "//shared:robot_constants",
        "//software/ai/passing:eighteen_zone_pitch_division",
        "//software/ai/passing:pass",
        "//software/ai/passing:pass_evaluation",
        "//software/ai/passing:pass_generator",
        "//software/estop:threaded_estop_reader",
        "//software/geom:angle",
        "//software/geom:point",
        "//software/geom:segment",
        "//software/geom:vector",
        "//software/geom/algorithms",
        "//software/networking:threaded_proto_udp_listener",
        "//software/networking:threaded_proto_udp_sender",
        "//software/uart:boost_uart_communication",
        "//software/world",
        "//software/world:field",
        "@pybind11_protobuf//pybind11_protobuf:native_proto_caster",
    ],
)

pybind_extension(
    name = "python_bindings",
    deps = [
        ":python_bindings_lib",
    ],
)
