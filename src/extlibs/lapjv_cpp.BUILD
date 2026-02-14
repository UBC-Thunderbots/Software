# Description:
#   This library provides us with an implementation of the LAPJV algorithm
#   https://en.wikipedia.org/wiki/Hungarian_algorithm
#   https://github.com/yongyanghz/LAPJV-algorithm-c/

cc_library(
    name = "lapjv_cpp",
    hdrs = glob(
        [
            "src/**/*.h",
            "src/**/*.cpp",
        ],
    ),
    include_prefix = "lapjv/",
    includes = [
        "./src",
    ],
    strip_include_prefix = "src/",
    visibility = ["//visibility:public"],
)
