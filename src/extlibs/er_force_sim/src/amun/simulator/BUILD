load("@bazel_rules_qt//:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        ":simulator_qt",
        "//extlibs/er_force_sim/src/core",
        "//extlibs/er_force_sim/src/protobuf",
    ],
)

qt_cc_library(
    name = "simulator_qt",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    linkstatic = True,
    deps = [
        "//extlibs/er_force_sim/src/core",
        "//extlibs/er_force_sim/src/protobuf",
        "@bullet",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)
