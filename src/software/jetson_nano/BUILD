cc_binary(
    name = "motor_interface",
    srcs = ["motor_interface.c"],
    copts = ["-pthread"],
    linkopts = ["-pthread", "-lrt"],
    deps = [
        "@trinamic"
    ],
)

cc_library(
    name = "primitive_executor",
    srcs = ["primitive_executor.cpp"],
    hdrs = ["primitive_executor.h"],
    deps = [
        "//proto/primitive:primitive_msg_factory",
        "//software/world",
        "//shared:constants",
        "//firmware/app/control:trajectory_planner",
        "//proto:tbots_cc_proto",
    ],
)
