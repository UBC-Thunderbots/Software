package(default_visibility = ["//visibility:public"])

filegroup(
    name = "libs",
    srcs = glob(["usr/lib/gcc/x86_64-linux-gnu/7.4.0/*.a"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/include/**",
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/include",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = [
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/libstdc++.so",
    ],
)

filegroup(
    name = "static_libs",
    srcs = [
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/libstdc++.a",
    ],
)
