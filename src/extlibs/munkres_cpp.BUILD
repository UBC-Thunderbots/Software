# Description:
#   This library provides us with an implementation of the Hungarian algorithm
#   https://en.wikipedia.org/wiki/Hungarian_algorithm
#   https://github.com/saebyn/munkres-cpp

cc_library(
    name = "munkres_cpp",
    # This library is entirely header based, even the `.cpp` files are really headers
    hdrs = glob(
        [
            "src/**/*.h",
            "src/**/*.cpp",
        ],
    ),
    include_prefix = "munkres/",
    includes = [
        "./src",
        "./src/adapters",
    ],
    strip_include_prefix = "src/",
    visibility = ["//visibility:public"],
)
