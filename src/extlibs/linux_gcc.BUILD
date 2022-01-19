package(default_visibility = ["//visibility:public"])

filegroup(
    name = "libs",
    srcs = glob(["usr/lib/gcc/x86_64-linux-gnu/11/*.a"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "usr/lib/gcc/x86_64-linux-gnu/11/include/**",
        "usr/lib/gcc/x86_64-linux-gnu/11/include",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = [
        "usr/lib/gcc/x86_64-linux-gnu/11/libstdc++.so",
    ],
)

filegroup(
    name = "static_libs",
    srcs = [
        "usr/lib/gcc/x86_64-linux-gnu/11/libstdc++.a",
    ],
)
