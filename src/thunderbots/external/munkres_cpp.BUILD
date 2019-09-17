cc_library(
    name = "all",
    # This library is entirely header based, even the `.cpp` files are really headers
    hdrs = glob(
        ["src/**/*.h", "src/**/*.cpp"],
    ),
    include_prefix = "munkres/",
    strip_include_prefix = "src/",
    includes = ["./src", "./src/adapters"],
    visibility = ["//visibility:public"],
)
