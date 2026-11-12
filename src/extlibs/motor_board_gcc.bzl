package(default_visibility = ["//visibility:public"])

filegroup(
    name = "includes",
    srcs = glob([
        "arm-none-eabi/include",
        "lib/gcc/arm-none-eabi/14.3.1/include",
    ]),
)
