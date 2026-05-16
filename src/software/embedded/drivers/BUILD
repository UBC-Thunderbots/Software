load("@ansible_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

py_binary(
    name = "flash_motor_drivers",
    srcs = [
        "flash_motor_drivers.py",
    ],
    deps = [
        requirement("gpiozero"),
    ],
)
