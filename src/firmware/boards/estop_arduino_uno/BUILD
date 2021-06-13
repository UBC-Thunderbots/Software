package(default_visibility = ["//visibility:public"])

#example build file which shows how arduino targets can be built and flashed to the arduino directly
#building the platformio_project will compile the Arduino code, running the platform_io project will flash the
#connected Arduino

#more information can be found here: https://github.com/mum4k/platformio_rules/blob/master/docs/platformio_doc.md

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
