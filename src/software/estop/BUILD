package(default_visibility = ["//visibility:public"])

cc_library(
    name = "estopManager",
    srcs = ["estopManager.cpp"],
    hdrs = ["estopManager.h"],
    deps = [
        "arduinoSerialWrapper",
        "//software/util/make_enum",
    ],
)

cc_library(
    name = "arduinoSerialWrapper",
    srcs = ["UARTCommunication.cpp"],
    hdrs = ["UARTCommunication.h"],
    deps = ["@boost//:asio"],
)

cc_binary(
    name = "estopMain",
    srcs = ["estopMain.cpp"],
    deps = [
        "arduinoSerialWrapper",
        "estopManager",
    ],
)
