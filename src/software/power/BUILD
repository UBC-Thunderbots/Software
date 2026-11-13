load("@bazel_skylib//rules:common_settings.bzl", "bool_flag")
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

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
    name = "power_hal",
    hdr = "power_hal.h",
)

cc_library(
    name = "power_hal_cc",
    hdrs = ["power_hal.h"],
    includes = ["."],
)

cc_library(
    name = "sim_hal",
    srcs = ["sim/sim_hal.cpp"],
    hdrs = ["sim/sim_hal.h"],
    includes = ["sim"],
    deps = [":power_hal_cc"],
)

cc_library(
    name = "power_hal_cc",
    hdrs = ["power_hal.h"],
    includes = ["."],
)

platformio_library(
    name = "power_command_handler",
    hdr = "power_command_handler.h",
    deps = ["//proto:tbots_nanopb_proto"],
)

cc_library(
    name = "power_command_handler_cc",
    hdrs = ["power_command_handler.h"],
    includes = ["."],
    deps = ["//proto:tbots_nanopb_proto"],
)

platformio_library(
    name = "esp_hal",
    src = "esp/esp_hal.cpp",
    add_srcs = [
        "esp/charger.cpp",
        "esp/chicker.cpp",
        "esp/dribbler.cpp",
        "esp/power_monitor.cpp",
    ],
    hdr = "esp/esp_hal.h",
    deps = [
        ":pins",
        ":power_hal",
        "@LTC4151//:LTC4151_platformio",
    ],
)

platformio_library(
    name = "power_controller",
    src = "power_controller.cpp",
    hdr = "power_controller.h",
    deps = [":power_hal"],
)

cc_library(
    name = "power_controller_cc",
    srcs = ["power_controller.cpp"],
    hdrs = ["power_controller.h"],
    includes = ["."],
    deps = [":power_hal_cc"],
)

platformio_library(
    name = "dribbler_controller",
    src = "dribbler_controller.cpp",
    hdr = "dribbler_controller.h",
    deps = [":power_hal"],
)

cc_library(
    name = "dribbler_controller_cc",
    srcs = ["dribbler_controller.cpp"],
    hdrs = ["dribbler_controller.h"],
    includes = ["."],
    deps = [":power_hal_cc"],
)

platformio_library(
    name = "power_command_receiver",
    src = "power_command_receiver.cpp",
    hdr = "power_command_receiver.h",
    deps = [
        "//proto:tbots_nanopb_proto",
        "//shared/uart_framing:uart_framing_platformio",
    ],
)

cc_library(
    name = "power_command_receiver_cc",
    srcs = ["power_command_receiver.cpp"],
    hdrs = ["power_command_receiver.h"],
    includes = ["."],
    deps = [
        "//proto:tbots_nanopb_proto",
        "//shared/uart_framing:uart_framing",
    ],
)

platformio_library(
    name = "control_executor",
    src = "control_executor.cpp",
    hdr = "control_executor.h",
    deps = [
        ":power_controller",
        ":power_command_handler",
        "//proto:tbots_nanopb_proto",
    ],
)

cc_library(
    name = "control_executor_cc",
    srcs = ["control_executor.cpp"],
    hdrs = ["control_executor.h"],
    includes = ["."],
    deps = [
        ":power_controller_cc",
        ":power_command_handler_cc",
        "//proto:tbots_nanopb_proto",
    ],
)

platformio_library(
    name = "power_board_runtime",
    src = "power_board_runtime.cpp",
    hdr = "power_board_runtime.h",
    deps = [
        ":dribbler_controller",
        ":power_command_handler",
        ":power_command_receiver",
        ":power_hal",
        "//proto/message_translation:power_frame_msg_platformio",
        "//shared/uart_framing:uart_framing_platformio",
    ],
)

cc_library(
    name = "power_board_runtime_cc",
    srcs = ["power_board_runtime.cpp"],
    hdrs = ["power_board_runtime.h"],
    includes = ["."],
    deps = [
        ":dribbler_controller_cc",
        ":power_command_handler_cc",
        ":power_command_receiver_cc",
        ":power_hal_cc",
        "//proto/message_translation:power_frame_msg",
        "//shared/uart_framing:uart_framing",
    ],
)

platformio_library(
    name = "pins",
    hdr = "esp/pins.h",
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
    platform = "espressif32@7.1.0\nupload_port = /dev/ttyAMA0\nboard_upload.before_reset = no_reset\nupload_speed = 115200",
    deps = [
        ":control_executor",
        ":dribbler_controller",
        ":esp_hal",
        ":pins",
        ":power_board_runtime",
        ":power_controller",
        ":power_hal",
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
