# Description:
#   G3log is a logging library that allows us to publish at multiple logging levels,
#   log to custom sinks, etc.

# G3log uses cmake to generate some files, so we need to run it
# before we build
genrule(
    name = "g3log_cmake",
    outs = ["include/g3log/generated_definitions.hpp"],
    srcs = glob(["**/*"]),
    cmd = "cd external/g3log; mkdir -p $(@D); && cmake .",
)

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
    ) + ["include/g3log/generated_definitions.hpp"],
    includes = ["./src"],
    visibility = ["//visibility:public"],
)
