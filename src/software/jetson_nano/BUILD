cc_binary(
    name = "motor_interface",
    srcs = ["motor_interface.c"],
    copts = ["-pthread"],
    linkopts = ["-pthread", "-lrt"],
    deps = [
        "@trinamic"
    ],
)
