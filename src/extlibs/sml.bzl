# Description:
#   This library provides us with a state machine library
#   https://github.com/boost-ext/sml
#   https://boost-ext.github.io/sml/tutorial.html

load("@rules_cc//cc:cc_library.bzl", "cc_library")

# Expose the repository-pinned header for the PlatformIO packaging target.
exports_files(["include/boost/sml.hpp"])

cc_library(
    name = "sml",
    hdrs = ["include/boost/sml.hpp"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
