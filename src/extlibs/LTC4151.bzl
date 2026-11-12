load(
    "@platformio_rules//platformio:platformio.bzl",
    "platformio_library",
)

package(default_visibility = ["//visibility:public"])

platformio_library(
    name = "LTC4151_platformio",
    src = "LTC4151.cpp",
    add_hdrs = ["LTC4151.h"],
    add_srcs = ["LTC4151.cpp"],
    hdr = "LTC4151.h",
)
