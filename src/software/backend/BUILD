package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    srcs = ["backend.cpp"],
    hdrs = ["backend.h"],
    deps = [
        "//proto:sensor_msg_cc_proto",
        "//proto/message_translation:tbots_protobuf",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/world",
    ],
)

cc_library(
    # TODO (#2510) Rename to SimulatorBackend
    name = "unix_simulator_backend",
    srcs = ["unix_simulator_backend.cpp"],
    hdrs = ["unix_simulator_backend.h"],
    deps = [
        ":backend",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:ssl_wrapper",
        "//proto/message_translation:tbots_protobuf",
        "//shared:constants",
        "//software:constants",
        "//software/logger",
        "//software/networking:threaded_proto_unix_listener",
        "//software/networking:threaded_proto_unix_sender",
        "//software/util/generic_factory",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "all_backends",
    deps = [
        ":unix_simulator_backend",
    ],
)
