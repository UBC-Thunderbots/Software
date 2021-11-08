genrule(
    name = "hiredis_cmake",
    srcs = glob(["**/*"]),
    outs = ["libhiredis.a"],
    cmd = "\n".join([
        # Run make, silencing both stdout (">") and stderr ("2>")
        "make -C external/hiredis_library",
        # Copy the generated header to the location bazel expects it
        "mv external/hiredis_library/libhiredis.a $@",
    ]),
)


cc_library(
    name = "hiredis_library",
    textual_hdrs = ["dict.c"],
    srcs = glob(
        ["*.c"],
        exclude=["dict.c"],
    ),
    hdrs = glob(
        ["*.h"],
    ),

    visibility = ["//visibility:public"],
)
