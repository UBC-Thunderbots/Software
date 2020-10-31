cc_library(
    name = "g3sinks",
    deps = [
        "@g3log",
        "@boost//:filesystem",
        "@zlib"
    ],
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
)
