# Description:
#   This library provides us with an implementation of the Hungarian algorithm
#   https://en.wikipedia.org/wiki/Hungarian_algorithm

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
