package(default_visibility = ["//visibility:public"])

load(":cc_toolchain_config.bzl", "cc_toolchain_config_jetson_nano", "cc_toolchain_config_k8", "cc_toolchain_config_arm")

# Create environments for each CPU this toolchain supports
environment(name = "k8")

environment(name = "jetson_nano")

environment(name = "aarch64")

environment_group(
    name = "cpus",
    defaults = [
        ":k8",
        ":jetson_nano",
        ":aarch64",
    ],
    environments = [
        ":k8",
        ":jetson_nano",
        ":aarch64",
    ],
)

config_setting(
    name = "cpu_k8",
    values = {"cpu": "k8"},
)

config_setting(
    name = "cpu_jetson_nano",
    values = {"cpu": "jetson_nano"},
)

config_setting(
    name = "cpu_aarch64",
    values = {"cpu": "aarch64"},
)

# This represents a mapping of CPU -> Compiler To Use
# The CPU can be chosen by using `--cpu` as a bazel flag (ex. `--cpu=jetson_nano`)
# otherwise it will default to using whatever CPU is in the system doing the compiling
cc_toolchain_suite(
    name = "toolchain",
    toolchains = {
        # k8 is any x86 system
        "k8": "cc_toolchain_linux_gcc",
        "k8|compiler": "cc_toolchain_linux_gcc",
        # jetson nano 4GB system
        "jetson_nano": "cc_toolchain_jetson_nano",
        "jetson_nano|compiler": "cc_toolchain_jetson_nano",
        "aarch64": "cc_toolchain_linux_gcc_arm",
        "aarch64|compiler": "cc_toolchain_linux_gcc_arm"
    },
)

filegroup(
    name = "empty",
    srcs = [],
)

filegroup(
    name = "jetson_nano_everything",
    srcs = [
        "@jetson_nano_gcc//:everything",
    ] + glob(["wrapper/jetson_nano_*"]),
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

cc_toolchain(
    name = "cc_toolchain_linux_gcc_arm",
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
    toolchain_config = ":linux_gcc_arm",
    toolchain_identifier = "aarch64_gcc-arm",
)

cc_toolchain_config_arm(
    name = "linux_gcc_arm",
    builtin_include_directories = [
        "/usr/lib/gcc/aarch64-linux-gnu/",
        "/usr/include/",
        "/usr/local/include/",
        "/usr/lib/gcc/c++/*/include/",
    ],
    cpu = "aarch64",
    target_cpu = "aarch64",
    target_system_name = "aarch64",
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
    toolchain_identifier = "linux_gcc-arm",
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
    tool_paths = {
        "ar": "wrapper/jetson_nano_ar",
        "cpp": "wrapper/jetson_nano_cpp",
        "dwp": "wrapper/jetson_nano_dwp",
        "gcc": "wrapper/jetson_nano_gcc",
        "gcov": "wrapper/jetson_nano_gcov",
        "ld": "wrapper/jetson_nano_ld",
        "nm": "wrapper/jetson_nano_nm",
        "objcopy": "wrapper/jetson_nano_objcopy",
        "objdump": "wrapper/jetson_nano_objdump",
        "strip": "wrapper/jetson_nano_strip",
    },
    toolchain_identifier = "gcc-jetson_nano",
)
