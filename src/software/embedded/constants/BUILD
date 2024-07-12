package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constants",
    hdrs = [
        "constants.h",
        "jetson_constants.h",
        "pi_constants.h",
    ],
    deps = [
        "//software/embedded:platform"
    ],
    defines = select({
        "//software/embedded:build_nano": ["NANO"],
        "//software/embedded:build_pi": ["PI"],
    }),
)
