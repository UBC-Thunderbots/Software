package(default_visibility = ["//visibility:public"])

load(":cc_toolchain_config.bzl", "cc_toolchain_config_k8", "cc_toolchain_config_stm32")

# Create environments for each CPU this toolchain supports
environment(name = "k8")

environment(name = "stm32f4")

environment(name = "stm32h7")

environment_group(
    name = "cpus",
    defaults = [
        ":k8",
        ":stm32h7",
        ":stm32f4",
    ],
    environments = [
        ":k8",
        ":stm32h7",
        ":stm32f4",
    ],
)

config_setting(
    name = "cpu_k8",
    values = {"cpu": "k8"},
)

config_setting(
    name = "cpu_stm32f4",
    values = {"cpu": "stm32f4"},
)

config_setting(
    name = "cpu_stm32h7",
    values = {"cpu": "stm32h7"},
)

# This represents a mapping of CPU -> Compiler To Use
# The CPU can be chosen by using `--cpu` as a bazel flag (ex. `--cpu=stm32h7`)
# otherwise it will default to using whatever CPU is in the system doing the compiling
cc_toolchain_suite(
    name = "toolchain",
    toolchains = {
        # k8 is any x86 system
        "k8": "cc_toolchain_linux_gcc",
        "k8|compiler": "cc_toolchain_linux_gcc",
        # stm32f4 MCU
        "stm32f4": "cc_toolchain_stm32f4",
        "stm32f4|compiler": "cc_toolchain_stm32f4",
        # stm32h7 MCU
        "stm32h7": "cc_toolchain_stm32h7",
        "stm32h7|compiler": "cc_toolchain_stm32h7",
    },
)

filegroup(
    name = "empty",
    srcs = [],
)

filegroup(
    name = "arm_everything",
    srcs = [
        "@arm_developer_gcc//:everything",
    ] + glob(["wrapper/arm-none-eabi-*"]),
)

filegroup(
    name = "k8_gcc_wrappers",
    srcs = glob(["wrapper/k8_gcc-*"]),
)

filegroup(
    name = "k8_gcc_all",
    srcs = [
        ":k8_gcc_wrappers",
        "@k8_gcc//:binaries",
        "@k8_gcc//:k8_gcc_libs",
        "@k8_gcc//:includes",
        "@k8_gcc//:runtime_libs",
        "@k8_gcc//:static_libs",
    ],
)

cc_toolchain_config_k8(
    name = "k8_gcc",
    builtin_include_directories = [
        "/usr/lib/gcc/x86_64-linux-gnu/",
        "/usr/include/",
        "/usr/local/include/",
        "/usr/lib/gcc/c++/7.4.0/include/",
    ],
    cpu = "k8",
    host_compiler_path = "wrapper/k8_gcc-k8_gcc",
    host_compiler_prefix = "wrapper",
    target_cpu = "k8",
    target_system_name = "k8",
    toolchain_identifier = "k8_gcc-k8",
)

cc_toolchain(
    name = "cc_toolchain_linux_gcc",
    all_files = ":k8_gcc_all",
    ar_files = ":k8_gcc_all",
    as_files = ":k8_gcc_all",
    compiler_files = ":k8_gcc_all",
    dwp_files = ":k8_gcc_all",
    dynamic_runtime_lib = "@k8_gcc//:runtime_libs",
    linker_files = ":k8_gcc_all",
    objcopy_files = ":k8_gcc_all",
    static_runtime_lib = "@k8_gcc//:static_libs",
    strip_files = ":k8_gcc_all",
    supports_param_files = True,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":k8_gcc",
    toolchain_identifier = "k8_gcc-x86_64",
)

cc_toolchain_config_stm32(
    name = "gcc-stm32f4",
    builtin_include_directories = [
        "external/arm_developer_gcc/arm-none-eabi/include/",
        "external/arm_developer_gcc/arm-none-eabi/include/sys/",
    ],
    cpu = "stm32f4",
    host_compiler_path = "wrapper/arm-none-eabi-g++",
    host_compiler_prefix = "wrapper",
    toolchain_identifier = "gcc-stm32f4",
)

cc_toolchain(
    name = "cc_toolchain_stm32f4",
    all_files = ":arm_everything",
    ar_files = ":arm_everything",
    as_files = ":arm_everything",
    compiler_files = ":arm_everything",
    dwp_files = ":empty",
    linker_files = ":arm_everything",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":gcc-stm32f4",
    toolchain_identifier = "gcc-stm32f4",
)

cc_toolchain_config_stm32(
    name = "gcc-stm32h7",
    builtin_include_directories = [
        "external/arm_developer_gcc/arm-none-eabi/include/",
        "external/arm_developer_gcc/arm-none-eabi/include/sys/",
    ],
    cpu = "stm32h7",
    host_compiler_path = "wrapper/arm-none-eabi-g++",
    host_compiler_prefix = "wrapper",
    toolchain_identifier = "gcc-stm32h7",
)

cc_toolchain(
    name = "cc_toolchain_stm32h7",
    all_files = ":arm_everything",
    ar_files = ":arm_everything",
    as_files = ":arm_everything",
    compiler_files = ":arm_everything",
    dwp_files = ":empty",
    linker_files = ":arm_everything",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":gcc-stm32h7",
    toolchain_identifier = "gcc-stm32h7",
)
