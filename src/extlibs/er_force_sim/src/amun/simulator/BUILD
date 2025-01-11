package(default_visibility = ["//visibility:public"])

cc_library(
    name = "simulator",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.h"]),
    deps = [
        "//extlibs/er_force_sim/src/core",
        "//extlibs/er_force_sim/src/protobuf",
        "//proto:ssl_simulation_cc_proto",
        "//shared:constants",
        "@bullet",
    ],
    alwayslink = True,
)
