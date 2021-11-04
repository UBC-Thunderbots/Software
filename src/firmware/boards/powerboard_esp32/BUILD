package(default_visibility = ["//visibility:public"])

# building the platformio_project will compile the Arduino code, running the platform_io project will flash the PICO
# more information can be found here: https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md

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
    name = "main",
    src = "main.cc",
    board = "pico32",
    framework = "arduino",
    platform = "espressif32",
    deps = [
        ":echo_library",
        "//shared:constants_platformio",
    ],
)
