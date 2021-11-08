genrule(
    name = "redis_cmake",
    srcs = glob(["**/*"]),
    outs = ["libredis++.a"],
    cmd = "\n".join([
        # Run make, silencing both stdout (">") and stderr ("2>")
        "cd external/redis-plus-plus",
        "mkdir build",
        "cd build",
        "cmake -DCMAKE_PREFIX_PATH=../../external/hiredis_library -DCMAKE_INSTALL_PREFIX=../redis-plus-plus ..",
        "make",   
        # Copy the generated header to the location bazel expects it
        "mv libredis++.a $@",
    ]),
)

cc_library(
    name = "redis-plus-plus",
    srcs = glob(
        [
            "src/sw/redis++/*.c"
        ]
    ) + ["libredis++.a"],
    hdrs = glob(
        ["src/sw/redis++/*.h"],
    ),
    deps = ["@hiredis_library"],
    visibility = ["//visibility:public"],
)

