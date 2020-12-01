package(default_visibility = ["//visibility:public"])

load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

platformio_library(
    name = "echo_library",
    src = "echo_library.cc",
    hdr = "echo_library.h",
)

platformio_project(
    name = "example_arduino_project",
    src = "serial_echo_example.cc",
    board = "uno",
    framework = "arduino",
    deps = [
        ":echo_library",
    ],
)
