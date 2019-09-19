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
    visibility = ["//visibility:public"],
)
