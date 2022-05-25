package(default_visibility = ["//software/gui:__subpackages__"])

cc_library(
    name = "geometry_conversion",
    srcs = ["geometry_conversion.cpp"],
    hdrs = ["geometry_conversion.h"],
    deps = [
        "//software/geom:point",
        "//software/geom:rectangle",
        "//software/geom:segment",
        "@qt//:qt_core",
        "@qt//:qt_gui",
    ],
)

cc_test(
    name = "geometry_conversion_test",
    srcs = ["geometry_conversion_test.cpp"],
    deps = [
        ":geometry_conversion",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
