# Description:
#   This library provides us with abstractions for working with USB devices

cc_library(
    name = "all",
    srcs = glob([
        "libusb/*.c",
        "libusb/os/linux_*.c",
        "libusb/os/*_posix.c",
    ]),
    hdrs = glob([
        "libusb/*.h",
        "libusb/os/linux_*.h",
        "libusb/os/*_posix.h",
        "Xcode/config.h",
    ]),
    includes = [
        ".",
        "Xcode",
        "libusb",
        "libusb/os",
    ],
    deps = [
        "@libudev//:all",
    ],
    visibility = ["//visibility:public"],
)
