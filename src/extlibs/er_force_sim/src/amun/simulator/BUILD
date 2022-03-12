load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

qt_cc_library(
    name = "simulator_qt",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//extlibs/er_force_sim/src/core:core_qt",
        "//extlibs/er_force_sim/src/protobuf:protobuf_qt",
        "//proto:ssl_simulation_cc_proto",
        "@bullet",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
    #linkstatic = True,
    alwayslink = True,
)
