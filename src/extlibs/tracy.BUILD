# Description
# Tracy profiler is a real-time, nanosecond resolution profiler. It is useful to profile our code when using debug
# symbols would slow down the code too much.
cc_library(
    name = "tracy",
    srcs = ["public/TracyClient.cpp"],
    hdrs = glob(
        [
            "public/**/*.h",
            "public/**/*.hpp",
            "public/**/*.cpp",
        ],
    ),
    includes = ["public/tracy"],
    visibility = ["//visibility:public"],
)
