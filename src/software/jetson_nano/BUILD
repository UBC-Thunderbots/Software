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
    hdrs = ["primitive_executor.h"],
    deps = [
        "//software/world",
        "//proto:tbots_cc_proto",
    ],
)

cc_library(
    name = "velocity_trajectory_primitive_executor",
    srcs = ["velocity_trajectory_primitive_executor.cpp"],
    hdrs = ["velocity_trajectory_primitive_executor.h"],
    deps = [
        ":primitive_executor",
        "//software/world",
        "//shared:constants",
        "//firmware/app/control:trajectory_planner",
        "//proto:tbots_cc_proto",
    ],
)
