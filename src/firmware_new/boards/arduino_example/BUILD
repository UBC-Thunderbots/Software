package(default_visibility = ["//visibility:public"])

load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

platformio_library(
    name = "echo_function",
    src = "echo_function.cc",
    hdr = "echo_function.h",
)

platformio_project(
    name = "example_arduino_echo_project",
    src = "serialEchoExample.cc",
    board = "uno",
    framework = "arduino",
    deps = [
        ":echo_function",
    ],
)
