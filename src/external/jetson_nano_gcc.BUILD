package(default_visibility = ["//visibility:public"])

filegroup(
    name = "everything",
    srcs = glob(["**"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "include/**",
        "aarch64-linux-gnu/include/**",
        "lib/gcc/aarch64-linux-gnu/7.3.1/include/**",
        "lib/gcc/aarch64-linux-gnu/7.3.1/include-fixed/**",
        "aarch64-linux-gnu/libc/usr/include/**",
        "include",
        "aarch64-linux-gnu/include",
        "aarch64-linux-gnu/7.3.1/include",
        "aarch64-linux-gnu/libc/usr/include",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = glob([
        "aarch64-linux-gnu/libc/lib/*.so.*",
        "aarch64-linux-gnu/libc/usr/lib/*.so.*",
        "aarch64-linux-gnu/lib/*.so.*",
        "aarch64-linux-gnu/libc/lib/*.so",
        "aarch64-linux-gnu/libc/usr/lib/*.so",
        "aarch64-linux-gnu/lib/*.so",
    ]),
)

filegroup(
    name = "static_libs",
    srcs = glob([
        "aarch64-linux-gnu/libc/lib/*.a",
        "aarch64-linux-gnu/libc/usr/lib/*.a",
        "aarch64-linux-gnu/lib/*.a",
    ]),
)
