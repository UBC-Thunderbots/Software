# BUILD file for the g3log external repository.
#
# g3log is a CMake project and ships no Bazel support, so this build file
# reproduces the relevant parts of its CMake build (Build.cmake,
# GenerateMacroDefinitionsFile.cmake, Options.cmake) using default options.

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

# g3log's CMake writes this header into the build tree; it is #included as
# "g3log/generated_definitions.hpp". Reproduce the default CMake output
# (Options.cmake: CHANGE_G3LOG_DEBUG_TO_DBUG=OFF -> "G3LOG_DEBUG DEBUG").
genrule(
    name = "generated_definitions",
    outs = ["g3log/generated_definitions.hpp"],
    cmd = "mkdir -p $(@D) && printf '#pragma once\\n\\n#define G3LOG_DEBUG DEBUG\\n' > $@",
)

# hermetic-llvm tags its musl platforms with this constraint. musl provides no
# <execinfo.h>/backtrace(), so build g3log without stack traces there.
config_setting(
    name = "musl",
    constraint_values = ["@llvm//constraints/libc:musl"],
)

# Non-Windows sources: everything under src/ except the Windows crash handler
# and Windows stacktrace (see Build.cmake).
cc_library(
    name = "g3log",
    srcs = [
        "src/crashhandler_unix.cpp",
        "src/filesink.cpp",
        "src/g3log.cpp",
        "src/logcapture.cpp",
        "src/loglevels.cpp",
        "src/logmessage.cpp",
        "src/logworker.cpp",
        "src/time.cpp",
        ":generated_definitions",
    ],
    hdrs = glob(
        ["src/g3log/*.hpp"],
        # Errors out on non-Windows platforms; never included there.
        exclude = ["src/g3log/stacktrace_windows.hpp"],
    ) + [":generated_definitions"],
    # "filesink.cpp" does #include "filesinkhelper.ipp".
    textual_hdrs = ["src/filesinkhelper.ipp"],
    # "src" for the g3log/*.hpp headers, "." for the generated header.
    includes = [
        ".",
        "src",
    ],
    defines = select({
        ":musl": ["G3LOG_DISABLE_BACKTRACE"],
        "//conditions:default": [],
    }),
    linkopts = ["-pthread"],
)
