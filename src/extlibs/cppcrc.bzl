# Description:
#   A very small, fast, header-only, C++ library for generating CRCs
#   https://github.com/DarrenLevine/cppcrc/tree/main

load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "cppcrc",
    hdrs = ["cppcrc.h"],
    visibility = ["//visibility:public"],
)
