package(default_visibility = ["//visibility:public"])

py_library(
    name = "screen",
    srcs = ["screen.py"],
    deps = [
        "//software/jetson_nano/display:constants",
        "//software/jetson_nano/display:utils",
    ],
)

py_library(
    name = "home_screen",
    srcs = ["home_screen.py"],
    deps = [
        ":screen",
    ],
)

py_library(
    name = "menu_screen",
    srcs = ["menu_screen.py"],
    deps = [
        ":screen",
    ],
)

py_library(
    name = "wheels_screen",
    srcs = ["wheels_screen.py"],
    deps = [
        ":screen",
    ],
)

py_library(
    name = "chip_and_kick_screen",
    srcs = ["chip_and_kick_screen.py"],
    deps = [
        ":screen",
    ],
)
