package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_util",
    testonly = True,
    srcs = [
        "equal_within_tolerance.cpp",
        "test_util.cpp",
    ],
    hdrs = [
        "equal_within_tolerance.h",
        "test_util.h",
    ],
    deps = [
        "//firmware/app/logger",
        "//shared:constants",
        "//shared/proto:tbots_cc_proto",
        "//shared/proto:tbots_nanopb_proto",
        "//software/geom:angle",
        "//software/geom:angular_velocity",
        "//software/geom:geom_constants",
        "//software/geom:point",
        "//software/geom:polygon",
        "//software/geom:vector",
        "//software/geom/algorithms",
        "//software/logger",
        "//software/simulation/physics:physics_world",
        "//software/world",
        "//software/world:robot_state",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "test_util_test",
    srcs = ["test_util_test.cpp"],
    deps = [
        ":test_util",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "equal_within_tolerance_test",
    srcs = ["equal_within_tolerance_test.cpp"],
    deps = [
        ":test_util",
        "@gtest//:gtest_main",
    ],
)
