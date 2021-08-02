package(default_visibility = ["//visibility:public"])

load(":cc_toolchain_config.bzl", "cc_toolchain_config_k8", "cc_toolchain_config_stm32", "cc_toolchain_config_jetson_nano")

# Create environments for each CPU this toolchain supports
environment(name = "k8")

environment(name = "stm32f4")

environment(name = "stm32h7")

environment(name = "jetson_nano")

environment_group(
    name = "cpus",
    defaults = [
        ":k8",
        ":jetson_nano",
        ":stm32h7",
        ":stm32f4",
    ],
    environments = [
        ":k8",
        ":jetson_nano",
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

config_setting(
    name = "cpu_jetson_nano",
    values = {"cpu": "jetson_nano"},
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
        # jetson nano 4GB system
        "jetson_nano": "cc_toolchain_jetson_nano",
        "jetson_nano|compiler": "cc_toolchain_jetson_nano",
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
    name = "jetson_nano_everything",
    srcs = [
        "@jetson_nano_gcc//:everything",
    ] + glob(["wrapper/jetson-nano-*"]) + glob(["wrapper/jetson_nano_*"]),
)

filegroup(
    name = "linux_gcc_wrapper",
    srcs = ["wrapper/linux_gcc"],
)

filegroup(
    name = "linux_gcc_all",
    srcs = [
        ":linux_gcc_wrapper",
        "@linux_gcc//:includes",
        "@linux_gcc//:libs",
        "@linux_gcc//:runtime_libs",
        "@linux_gcc//:static_libs",
    ],
)

cc_toolchain_config_k8(
    name = "linux_gcc",
    builtin_include_directories = [
        "/usr/lib/gcc/x86_64-linux-gnu/",
        "/usr/include/",
        "/usr/local/include/",
        "/usr/lib/gcc/c++/*/include/",
    ],
    cpu = "k8",
    target_cpu = "k8",
    target_system_name = "k8",
    tool_paths = {
        "ar": "/usr/bin/ar",
        "cpp": "/usr/bin/cpp",
        "dwp": "/usr/bin/dwp",
        "gcc": "wrapper/linux_gcc",
        "gcov": "/usr/bin/gcov",
        "ld": "/usr/bin/ld",
        "nm": "/usr/bin/nm",
        "objcopy": "/usr/bin/objcopy",
        "objdump": "/usr/bin/objdump",
        "strip": "/usr/bin/strip",
    },
    toolchain_identifier = "linux_gcc-k8",
)

cc_toolchain(
    name = "cc_toolchain_linux_gcc",
    all_files = ":linux_gcc_all",
    ar_files = ":linux_gcc_all",
    as_files = ":linux_gcc_all",
    compiler_files = ":linux_gcc_all",
    dwp_files = ":linux_gcc_all",
    dynamic_runtime_lib = "@linux_gcc//:runtime_libs",
    linker_files = ":linux_gcc_all",
    objcopy_files = ":linux_gcc_all",
    static_runtime_lib = "@linux_gcc//:static_libs",
    strip_files = ":linux_gcc_all",
    supports_param_files = True,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":linux_gcc",
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

cc_toolchain(
    name = "cc_toolchain_jetson_nano",
    all_files = ":jetson_nano_everything",
    ar_files = ":jetson_nano_everything",
    as_files = ":jetson_nano_everything",
    compiler_files = ":jetson_nano_everything",
    dwp_files = ":empty",
    linker_files = ":jetson_nano_everything",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":gcc-jetson_nano",
    toolchain_identifier = "gcc-jetson_nano",
)

cc_toolchain_config_jetson_nano(
    name = "gcc-jetson_nano",
    builtin_include_directories = [
        "external/jetson_nano_gcc/include/",
        "external/jetson_nano_gcc/aarch64-linux-gnu/include/",
        "external/jetson_nano_gcc/aarch64-linux-gnu/libc/lib/",
        "external/jetson_nano_gcc/aarch64-linux-gnu/libc/usr/include/",
    ],
    cpu = "jetson_nano",
    host_compiler_path = "wrapper/jetson_nano_gcc",
    host_compiler_prefix = "wrapper",
    toolchain_identifier = "gcc-jetson_nano",
)
