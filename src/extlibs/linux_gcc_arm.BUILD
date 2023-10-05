package(default_visibility = ["//visibility:public"])

filegroup(
    name = "libs",
    srcs = glob(["usr/lib/gcc/aarch64-linux-gnu/9/*.a"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "usr/lib/gcc/aarch64-linux-gnu/9/include/**",
        "usr/lib/gcc/aarch64-linux-gnu/9/include",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = [
        "usr/lib/gcc/aarch64-linux-gnu/9/libstdc++.so",
    ],
)

filegroup(
    name = "static_libs",
    srcs = [
        "usr/lib/gcc/aarch64-linux-gnu/9/libstdc++.a",
    ],
)
