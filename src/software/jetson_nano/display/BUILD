package(default_visibility = ["//visibility:public"])

load("@python_tools_deps//:requirements.bzl", "requirement")

py_binary(
    name = "rotary_encoder",
    srcs = ["rotary_encoder/rotary_encoder.py"],
    deps = [requirement("Jetson.GPIO")],
)

py_binary(
    name = "lcd_user_interface",
    srcs = ["lcd_user_inteface/lcd_user_inteface.py"],
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
