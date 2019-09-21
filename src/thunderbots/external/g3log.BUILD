# Description:
#   G3log is a logging library that allows us to publish at multiple logging levels,
#   log to custom sinks, etc.

cc_library(
    name = "all",
    srcs = glob(
        ["src/*.cpp", "src/*.ipp"],
        exclude = [
            "src/crashhandler_windows.cpp",
            "src/stacktrace_win.cpp",
            "src/stacktrace_windows.cpp",
        ],
    ),
    hdrs = glob(
        ["src/g3log/*.hpp"],
        exclude = ["src/g3log/stacktrace_windows.hpp"],
    ),
    includes = ["./src"],
    visibility = ["//visibility:public"],
)
