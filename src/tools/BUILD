package(default_visibility = ["//visibility:public"])

environment(name = "k8")

environment(name = "stm32h7")

environment_group(
    name = "cpus",
    defaults = [
        ":k8",
        ":stm32h7",
    ],
    environments = [
        ":k8",
        ":stm32h7",
    ],
)
