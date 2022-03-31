package(default_visibility = ["//visibility:public"])

load("@display_deps//:requirements.bzl", "requirement")

py_binary(
    name = "robot_ui",
    srcs = ["robot_ui.py"],
    deps = [
        "//software/display/lcd_user_interface",
        "//software/display/rotary_encoder",
        "//software/display/screens:chip_and_kick_screen",
        "//software/display/screens:home_screen",
        "//software/display/screens:menu_screen",
        "//software/display/screens:wheels_screen",
        requirement("Adafruit-Blinka"),
        requirement("adafruit-circuitpython-busdevice"),
        requirement("adafruit-circuitpython-rgb-display"),
        requirement("Adafruit-PlatformDetect"),
        requirement("Adafruit-PureIO"),
        requirement("Jetson.GPIO"),
        requirement("Pillow"),
        requirement("redis"),
    ],
)

py_library(
    name = "constants",
    srcs = ["constants.py"],
)

py_library(
    name = "utils",
    srcs = ["utils.py"],
)
