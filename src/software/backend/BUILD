package(default_visibility = ["//visibility:public"])

cc_library(
    name = "backend",
    hdrs = ["backend.h"],
    deps = [
        ":robot_status",
        "//software/ai/primitive",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/world",
    ],
)

cc_library(
    name = "backend_factory",
    srcs = ["backend_factory.cpp"],
    hdrs = ["backend_factory.h"],
    deps = [":backend"],
)

cc_test(
    name = "backend_factory_test",
    srcs = ["backend_factory_test.cpp"],
    deps = [
        ":backend_factory",
        "@gtest//:gtest_main",
    ],
)

cc_library(
    name = "grsim_backend",
    srcs = ["grsim_backend.cpp"],
    hdrs = ["grsim_backend.h"],
    deps = [
        ":backend",
        ":backend_factory",
        "//software/backend/input/network/networking:network_client",
        "//software/backend/output/grsim:grsim_output",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "radio_backend",
    srcs = ["radio_backend.cpp"],
    hdrs = ["radio_backend.h"],
    deps = [
        ":backend",
        ":backend_factory",
        "//software/backend/input/network/networking:network_client",
        "//software/backend/output/radio:radio_output",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "simulator_backend",
    srcs = ["simulator_backend.cpp"],
    hdrs = ["simulator_backend.h"],
    deps = [
        ":backend",
        "//software/backend/simulation:physics_simulator",
        "//software/util/logger",
        "//software/world",
    ],
    # We force linking so that the static variables required for the "factory"
    # design pattern to work are linked in
    # https://www.bfilipek.com/2018/02/static-vars-static-lib.html
    alwayslink = True,
)

cc_library(
    name = "robot_status",
    hdrs = ["robot_status.h"],
    deps = [],
)
