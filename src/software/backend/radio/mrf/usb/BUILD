package(default_visibility = ["//software/backend/radio/mrf:__subpackages__"])

cc_library(
    name = "libusb",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//software/backend/radio/mrf/util",
        "//software/logger",
        "@libusb",
    ],
)
