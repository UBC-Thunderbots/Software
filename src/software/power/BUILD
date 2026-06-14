load("@bazel_skylib//rules:common_settings.bzl", "bool_flag")
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

package(default_visibility = ["//visibility:public"])

# When enabled, compiles powerloop_main with the DEBUG_POWERLOOP flag, which
# swaps in the bare setup()/loop() stubs in powerloop_main.cc so arbitrary code
# can be flashed onto the powerboard microcontroller for debugging.
bool_flag(
    name = "debug_powerloop",
    build_setting_default = False,
)

config_setting(
    name = "debug_powerloop_enabled",
    flag_values = {
        ":debug_powerloop": "True",
    },
)

# building the platformio_project will compile the Arduino code, running the platform_io project will flash the PICO
# more information can be found here: https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md

platformio_library(
    name = "chicker",
    src = "chicker.cpp",
    hdr = "chicker.h",
    deps = [":pins"],
)

platformio_library(
    name = "dribbler",
    src = "dribbler.cpp",
    hdr = "dribbler.h",
    deps = [":pins"],
)

platformio_library(
    name = "geneva",
    src = "geneva.cpp",
    hdr = "geneva.h",
    deps = [
        ":pins",
        "//proto:tbots_nanopb_proto",
    ],
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
        "//proto:tbots_nanopb_proto",
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
    build_flags = ["-DPB_FIELD_32BIT=1"] + select({
        #Makes platformio use 16 bit numbers for size
        ":debug_powerloop_enabled": ["-DDEBUG_POWERLOOP"],
        "//conditions:default": [],
    }),
    framework = "arduino",
    lib_deps = ["Nanopb@^0.4.91"],
    platform = "espressif32\nupload_port = /dev/ttyAMA0\nboard_upload.before_reset = no_reset\nupload_speed = 115200",
    deps = [
        ":charger",
        ":chicker",
        ":control_executor",
        ":dribbler",
        ":geneva",
        ":pins",
        ":power_monitor",
        "//proto:tbots_nanopb_proto",
        "//shared:constants_platformio",
        "//shared/uart_framing:uart_framing_platformio",
    ],
)

# Integrates automatic bootloader mode / completion reset
genrule(
    name = "powerloop_tar",
    srcs = ["auto_reset.py"],
    outs = ["powerloop.tar.gz"],
    cmd = """
        TARGET_DIR=$$(dirname $(location :powerloop_main))
        cp $(location :auto_reset.py) "$$TARGET_DIR/powerloop_main_workdir/"
        chmod +w "$$TARGET_DIR/powerloop_main_workdir/platformio.ini"
        echo "" >> "$$TARGET_DIR/powerloop_main_workdir/platformio.ini"
        echo "extra_scripts = pre:auto_reset.py" >> "$$TARGET_DIR/powerloop_main_workdir/platformio.ini"
        tar -C "$$TARGET_DIR" -czhf $@ "powerloop_main_workdir"
    """,
    local = True,
    tools = [":powerloop_main"],
)
