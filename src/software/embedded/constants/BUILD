package(default_visibility = ["//visibility:public"])

cc_library(
    name = "constants",
    hdrs = [
        "constants.h",
        "jetson_constants.h",
        "pi_constants.h",
    ],
    defines = select({
        "//software/embedded:build_nano": ["NANO"],
        "//software/embedded:build_pi": ["PI"],
    }),
    deps = [
        "//software/embedded:platform",
    ],
)

py_library(
    name = "py_constants",
    srcs = [":py_constants.py"],
#    deps = [
#        requirement("protobuf"),
#    ],
)