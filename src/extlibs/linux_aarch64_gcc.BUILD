package(default_visibility = ["//visibility:public"])

filegroup(
    name = "libs",
    srcs = glob(["usr/lib/gcc/aarch64-linux-gnu/10/*.a"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "usr/lib/gcc/aarch64-linux-gnu/10/include/**",
        "usr/lib/gcc/aarch64-linux-gnu/10/include",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = [
        "usr/lib/gcc/aarch64-linux-gnu/10/libstdc++.so",
    ],
)

filegroup(
    name = "static_libs",
    srcs = [
        "usr/lib/gcc/aarch64-linux-gnu/10/libstdc++.a",
    ],
)
