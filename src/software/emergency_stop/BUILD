package(default_visibility = ["//visibility:public"])

# https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md
load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
    "platformio_project",
)

platformio_project(
    name = "main",
    src = "main.cc",
    board = "uno",
    framework = "arduino",
    deps = [
        "//shared:constants_platformio",
    ],
)
