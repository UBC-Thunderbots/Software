cc_library(
    name = "g3sinks",
    srcs = glob(
        [
            "logrotate/src/*.cpp",
            "logrotate/src/*.ipp",
        ],
    ),
    hdrs = glob(
        [
            "logrotate/src/g3sinks/*.h",
        ],
    ),
    includes = [
        "./logrotate/src",
    ],
    visibility = ["@//software/logger:__subpackages__"],
    deps = [
        "@boost//:filesystem",
        "@g3log",
        "@zlib",
    ],
)
