package(default_visibility = ["//visibility:public"])

# building the platformio_project will compile the Arduino code, running the platform_io project will flash the PICO
# more information can be found here: https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md

load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

platformio_library(
    name = "chicker",
    src = "chicker.cpp",
    hdr = "chicker.h",
    deps = [":pins"],
)

platformio_library(
    name = "geneva",
    src = "geneva.cpp",
    hdr = "geneva.h",
    deps = [":pins"],
)

platformio_library(
    name = "charger",
    src = "charger.cpp",
    hdr = "charger.h",
    deps = [":pins"],
)

platformio_library(
    name = "power_monitor",
    src = "power_monitor.cpp",
    hdr = "power_monitor.h",
    deps = [
        ":pins",
        "@LTC4151//:LTC4151_platformio",
    ],
)

platformio_library(
    name = "control_executor",
    src = "control_executor.cpp",
    hdr = "control_executor.h",
    deps = [
        ":charger",
        ":chicker",
        ":geneva",
        ":pins",
        "@nanopb//:nanopb_platformio",
    ],
)

platformio_library(
    name = "pins",
    hdr = "pins.h",
)

platformio_project(
    name = "powerloop_main",
    src = "powerloop_main.cc",
    board = "pico32",
    build_flags = ["-DPB_FIELD_16BIT=1"],  #Makes platformio use 16 bit numbers for size
    framework = "arduino",
    platform = "espressif32",
    deps = [
        ":charger",
        ":chicker",
        ":control_executor",
        ":geneva",
        ":pins",
        ":power_monitor",
        "//proto:tbots_nanopb_proto",
        "//shared:constants_platformio",
        "//shared/uart_framing:uart_framing_platformio",
        "@nanopb//:nanopb_platformio",
    ],
)

genrule(
    name = "powerloop_zip",
    outs = ["powerloop_zip.zip"],
    cmd = "zip -r $@ $(BINDIR)/../.*",
    tools = [":powerloop_main"],
)
