package(default_visibility = ["//visibility:public"])

py_binary(
    name = "robot_ui",
    srcs = ["robot_ui.py"],
    data = [
        "//software:py_constants.so",
    ],
    deps = [
        "//software/jetson_nano/display/lcd_user_interface",
        "//software/jetson_nano/display/rotary_encoder",
        "//software/jetson_nano/display/screens:chip_and_kick_screen",
        "//software/jetson_nano/display/screens:home_screen",
        "//software/jetson_nano/display/screens:menu_screen",
        "//software/jetson_nano/display/screens:wheels_screen",
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

genrule(
    name = "robot_ui_zip",
    outs = ["robot_ui_zip.zip"],
    cmd = "zip -r $@ $(location :robot_ui)*",
    tools = [":robot_ui"],
)
