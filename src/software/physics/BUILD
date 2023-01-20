package(default_visibility = ["//visibility:public"])

cc_library(
    name = "euclidean_to_wheel",
    srcs = [
        "euclidean_to_wheel.cpp",
    ],
    hdrs = [
        "euclidean_to_wheel.h",
    ],
    deps = [
        "//shared:robot_constants",
        "@eigen",
    ],
)

cc_test(
    name = "euclidean_to_wheel_test",
    srcs = [
        "euclidean_to_wheel_test.cpp",
    ],
    deps = [
        ":euclidean_to_wheel",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "velocity_conversion_util",
    hdrs = [
        "velocity_conversion_util.h",
    ],
    deps = [
        "//software/geom:vector",
    ],
)

cc_test(
    name = "velocity_conversion_util_test",
    srcs = [
        "velocity_conversion_util_test.cpp",
    ],
    deps = [
        ":velocity_conversion_util",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)

cc_library(
    name = "physics",
    srcs = [
        "physics.cpp",
    ],
    hdrs = [
        "physics.h",
    ],
    deps = [
        "//software/geom:point",
        "//software/time:duration",
        "@eigen",
    ],
)

cc_test(
    name = "physics_test",
    srcs = ["physics_test.cpp"],
    deps = [
        ":physics",
        "//shared/test_util:tbots_gtest_main",
        "//software/test_util",
    ],
)
