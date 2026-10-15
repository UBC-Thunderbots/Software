load("@rules_cc//cc:cc_library.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    # Clang's SLP vectorization (-O2/-O3) turns guarded scalar divisions in this
    # vendored er_force simulator code into packed divps instructions whose
    # zero-filled padding lanes evaluate 0/0. Our gtest harness unmasks
    # FE_INVALID (see shared/test_util/tbots_gtest_main.cpp), which turns those
    # dead lanes into a SIGFPE (observed in Simulator::Simulator). Disable SLP
    # vectorization for this library so it builds cleanly at -O3 (Bullet needs
    # the same treatment, see //extlibs:bullet.bzl).
    copts = ["-fno-slp-vectorize"],
    deps = [
        "//extlibs/er_force_sim/src/core",
        "//extlibs/er_force_sim/src/protobuf",
        "//proto:ssl_simulation_cc_proto",
        "//shared:constants",
        "@bullet",
    ],
    alwayslink = True,
)
