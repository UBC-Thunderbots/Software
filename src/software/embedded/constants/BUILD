package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constants",
    hdrs = [
        "constants.h",
        "jetson_constants.h",
        "limited_constants.h",
        "pi_constants.h",
    ],
    defines = select({
        "//software/embedded:build_nano": ["NANO"],
        "//software/embedded:build_pi": ["PI"],
        "//software/embedded:build_limited": ["LIMITED"],
    }),
    deps = [
        "//software/embedded:platform",
    ],
)
