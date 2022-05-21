package(default_visibility = ["//visibility:public"])

# building the platformio_project will compile the Arduino code, running the platform_io project will flash the PICO
# more information can be found here: https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md

load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

platformio_project(
    name = "powerloop_main",
    src = "powerloop_main.cc",
    board = "pico32",
    build_flags = ["-DPB_FIELD_16BIT=1"],  #Makes platformio use 16 bit numbers for size
    framework = "arduino",
    platform = "espressif32",
    deps = [
        "//proto:tbots_nanopb_proto",
        "//shared:constants_platformio",
        "//shared/uart_framing:uart_framing_platformio",
        "@nanopb//:nanopb_platformio",
    ],
)
