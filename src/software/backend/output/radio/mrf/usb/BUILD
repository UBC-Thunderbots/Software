package(
    default_visibility = ["//software/backend/output/radio/mrf:__subpackages__"],
    features = ["warnings_as_errors"],
)

cc_library(
    name = "libusb",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//software/backend/output/radio/mrf/util",
        "//software/logger",
        "@libusb",
    ],
)
