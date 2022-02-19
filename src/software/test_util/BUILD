package(default_visibility = ["//visibility:public"])

cc_library(
    name = "test_util",
    testonly = True,
    srcs = [
        "equal_within_tolerance.cpp",
        "test_util.cpp",
        "path_planning_test_util.cpp",
    ],
    hdrs = [
        "equal_within_tolerance.h",
        "test_util.h",
        "path_planning_test_util.h",
    ],
    deps = [
        "//firmware/app/logger",
        "//proto:tbots_cc_proto",
        "//proto:tbots_nanopb_proto",
        "//shared:constants",
        "//shared/test_util:tbots_gtest_main",
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
        "//software/ai/navigator/obstacle:obstacle",
        "@eigen",
    ],
)

cc_test(
    name = "test_util_test",
    srcs = ["test_util_test.cpp"],
    deps = [
        ":test_util",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_test(
    name = "equal_within_tolerance_test",
    srcs = ["equal_within_tolerance_test.cpp"],
    deps = [
        ":test_util",
        "//shared/test_util:tbots_gtest_main",
    ],
)
