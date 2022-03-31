package(default_visibility = ["//visibility:public"])

load("@display_deps//:requirements.bzl", "requirement")

py_binary(
    name = "rotary_encoder",
    srcs = ["rotary_encoder/rotary_encoder.py"],
    deps = [requirement("Jetson.GPIO")],
)

py_binary(
    name = "lcd_user_interface",
    srcs = ["lcd_user_interface/lcd_user_interface.py"],
    deps = [
        requirement("Adafruit-Blinka"),
        requirement("adafruit-circuitpython-busdevice"),
        requirement("adafruit-circuitpython-rgb-display"),
        requirement("Adafruit-PlatformDetect"),
        requirement("Adafruit-PureIO"),
        requirement("Jetson.GPIO"),
        requirement("Pillow"),
    ],
)

py_binary(
    name = "robot_ui",
    srcs = ["robot_ui.py"],
    deps = [
        "//software/display/rotary_encoder:rotary_encoder.py",
        "//software/display/lcd_user_interface:lcd_user_interface.py",
        "//software/display/screens:home_screen.py",
        "//software/display/screens:menu_screen.py",
        "//software/display/screens:wheels_screen.py",
        "//software/display/screens:chip_and_kick_screen.py",
    ],
)