# Description:
#   G3log is a logging library that allows us to publish at multiple logging levels,
#   log to custom sinks, etc.
#   https://github.com/KjellKod/g3log


load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

cmake(
    name = "g3log",
    lib_source = ":all_srcs",
    # Following flags are retrieved from g3log CMakeLists.txt
    # https://github.com/KjellKod/g3log/blob/2.4/CMakeLists.txt
    cache_entries = {
        "ADD_FATAL_EXAMPLE": "OFF",
        "ADD_G3LOG_BENCH_PERFORMANCE": "OFF",
        "ADD_G3LOG_UNIT_TEST": "OFF",
        "ADD_LOG_SAMPLE_SYSLOG": "OFF",
        "G3_SHARED_LIB": "OFF",
        # We need these two flags to make g3log compile
        # Otherwise, g3log will try to look for __cax_demangle
        # which cannot be found in our context. See
        # https://github.com/KjellKod/g3log/blob/38dbddc7071666ef6cc02f24ff30a8773ee4222f/Build.cmake#L120-L129
        "DEMANGLE_EXISTS": "ON",
    },
    visibility = ["//visibility:public"],
    out_static_libs = ["libg3log.a"],
)

filegroup(
    name = "all_srcs",
    srcs = glob([
        "**/*",
    ]),
)

