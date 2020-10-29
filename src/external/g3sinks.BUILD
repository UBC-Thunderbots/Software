# G3sinks uses cmake, to generate some files, so we need to run it
# before we build


cc_library(
    name = "g3sinks",
    srcs = glob(
        [
            "sink_logrotate/src/*.cpp",
            "sink_logrotate/src/*.ipp",
        ],
    ),
    hdrs = glob(
        ["sink_logrotate/src/g3sinks/*.hpp"],
    ),
    # We explicitly require this header so that bazel knows to run the genrule
    # above to create it
    includes = [
        "./include",
        "./src",
    ],
    visibility = ["@//software/logger:__subpackages__"],
)