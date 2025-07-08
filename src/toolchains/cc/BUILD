load(
    ":cc_toolchain_config.bzl",
    "cc_toolchain_config_fullsystem",
    "cc_toolchain_config_k8_jetson_nano_cross_compile",
    "make_builtin_include_directories",
)

package(default_visibility = ["//visibility:public"])

# Create environments for each CPU this toolchain supports
environment(name = "k8")

environment(name = "k8_jetson_nano_cross_compile")

environment(name = "aarch64")

environment_group(
    name = "cpus",
    defaults = [
        ":aarch64",
        ":k8",
        ":k8_jetson_nano_cross_compile",
    ],
    environments = [
        ":aarch64",
        ":k8",
        ":k8_jetson_nano_cross_compile",
    ],
)

config_setting(
    name = "cpu_aarch64",
    values = {"cpu": "aarch64"},
)

config_setting(
    name = "cpu_k8",
    values = {"cpu": "k8"},
)

config_setting(
    name = "cpu_k8_jetson_nano_cross_compile",
    values = {"cpu": "k8_jetson_nano_cross_compile"},
)

# Create a constraint for the glibc version to target the correct glibc version for the robot
constraint_setting(name = "glibc_version")

constraint_value(
    name = "glibc_2_27",
    constraint_setting = ":glibc_version",
)

platform(
    name = "robot",
    constraint_values = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
        ":glibc_2_27",
    ],
)

# This represents a mapping of CPU -> Compiler To Use
cc_toolchain_suite(
    name = "toolchain",
    toolchains = {
        # k8 is any x86 system
        "k8": "cc_toolchain_linux_k8_gcc",
        "k8|compiler": "cc_toolchain_linux_k8_gcc",
        # aarch64 is for ARM64 systems and the robot
        "aarch64": "cc_toolchain_linux_aarch64_gcc",
        "aarch64|compiler": "cc_toolchain_linux_aarch64_gcc",
        # jetson nano 4GB system
        "k8_jetson_nano_cross_compile": "cc_toolchain_k8_jetson_nano_cross_compile",
        "k8_jetson_nano_cross_compile|compiler": "cc_toolchain_k8_jetson_nano_cross_compile",
    },
)

filegroup(
    name = "empty",
    srcs = [],
)

filegroup(
    name = "k8_jetson_nano_cross_compile_everything",
    srcs = [
        "@k8_jetson_nano_cross_compile_gcc//:everything",
    ] + glob(["wrapper/k8_jetson_nano_cross_compile_*"]),
)

filegroup(
    name = "linux_gcc_wrapper",
    srcs = ["wrapper/linux_gcc"],
)

filegroup(
    name = "linux_k8_gcc_all",
    srcs = [
        ":linux_gcc_wrapper",
        "@linux_k8_gcc//:includes",
        "@linux_k8_gcc//:libs",
        "@linux_k8_gcc//:runtime_libs",
        "@linux_k8_gcc//:static_libs",
    ],
)

filegroup(
    name = "linux_aarch64_gcc_all",
    srcs = [
        ":linux_gcc_wrapper",
        "@linux_aarch64_gcc//:includes",
        "@linux_aarch64_gcc//:libs",
        "@linux_aarch64_gcc//:runtime_libs",
        "@linux_aarch64_gcc//:static_libs",
    ],
)

cc_toolchain_config_fullsystem(
    name = "linux_k8_gcc",
    builtin_include_directories = [
        "/usr/lib/gcc/x86_64-linux-gnu/",
    ] + make_builtin_include_directories(),
    host_system_name = "k8",
    target_cpu = "k8",
    target_system_name = "k8",
    toolchain_identifier = "linux_k8_gcc",
)

cc_toolchain_config_fullsystem(
    name = "linux_aarch64_gcc",
    builtin_include_directories = [
        "/usr/lib/gcc/aarch64-linux-gnu/",
    ] + make_builtin_include_directories(),
    host_system_name = "aarch64",
    target_cpu = "aarch64",
    target_system_name = "aarch64",
    toolchain_identifier = "linux_aarch64_gcc",
)

cc_toolchain(
    name = "cc_toolchain_linux_k8_gcc",
    all_files = ":linux_k8_gcc_all",
    ar_files = ":linux_k8_gcc_all",
    as_files = ":linux_k8_gcc_all",
    compiler_files = ":linux_k8_gcc_all",
    dwp_files = ":linux_k8_gcc_all",
    dynamic_runtime_lib = "@linux_k8_gcc//:runtime_libs",
    linker_files = ":linux_k8_gcc_all",
    objcopy_files = ":linux_k8_gcc_all",
    static_runtime_lib = "@linux_k8_gcc//:static_libs",
    strip_files = ":linux_k8_gcc_all",
    supports_param_files = True,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":linux_k8_gcc",
    toolchain_identifier = "k8_gcc",
)

cc_toolchain(
    name = "cc_toolchain_linux_aarch64_gcc",
    all_files = ":linux_aarch64_gcc_all",
    ar_files = ":linux_aarch64_gcc_all",
    as_files = ":linux_aarch64_gcc_all",
    compiler_files = ":linux_aarch64_gcc_all",
    dwp_files = ":linux_aarch64_gcc_all",
    dynamic_runtime_lib = "@linux_aarch64_gcc//:runtime_libs",
    linker_files = ":linux_aarch64_gcc_all",
    objcopy_files = ":linux_aarch64_gcc_all",
    static_runtime_lib = "@linux_aarch64_gcc//:static_libs",
    strip_files = ":linux_aarch64_gcc_all",
    supports_param_files = True,
    toolchain_config = ":linux_aarch64_gcc",
    toolchain_identifier = "aarch64_gcc",
)

cc_toolchain(
    name = "cc_toolchain_k8_jetson_nano_cross_compile",
    all_files = ":k8_jetson_nano_cross_compile_everything",
    ar_files = ":k8_jetson_nano_cross_compile_everything",
    as_files = ":k8_jetson_nano_cross_compile_everything",
    compiler_files = ":k8_jetson_nano_cross_compile_everything",
    dwp_files = ":empty",
    linker_files = ":k8_jetson_nano_cross_compile_everything",
    objcopy_files = ":empty",
    strip_files = ":empty",
    supports_param_files = 0,
    # We add this tag to circumvent a bug in the CLion bazel plugin. It should be removed
    # when possible
    # https://github.com/bazelbuild/intellij/issues/486
    tags = ["no-ide"],
    toolchain_config = ":gcc_k8_jetson_nano_cross_compile",
    toolchain_identifier = "gcc-k8-jetson-nano-cross-compile",
)

cc_toolchain_config_k8_jetson_nano_cross_compile(
    name = "gcc_k8_jetson_nano_cross_compile",
    builtin_include_directories = [
        "external/+_repo_rules3+k8_jetson_nano_cross_compile_gcc/include/",
        "external/+_repo_rules3+k8_jetson_nano_cross_compile_gcc/aarch64-linux-gnu/include/",
        "external/+_repo_rules3+k8_jetson_nano_cross_compile_gcc/aarch64-linux-gnu/libc/lib/",
        "external/+_repo_rules3+k8_jetson_nano_cross_compile_gcc/aarch64-linux-gnu/libc/usr/include/",
    ],
    tool_paths = {
        "ar": "wrapper/k8_jetson_nano_cross_compile_ar",
        "cpp": "wrapper/k8_jetson_nano_cross_compile_cpp",
        "dwp": "wrapper/k8_jetson_nano_cross_compile_dwp",
        "gcc": "wrapper/k8_jetson_nano_cross_compile_gcc",
        "gcov": "wrapper/k8_jetson_nano_cross_compile_gcov",
        "ld": "wrapper/k8_jetson_nano_cross_compile_ld",
        "nm": "wrapper/k8_jetson_nano_cross_compile_nm",
        "objcopy": "wrapper/k8_jetson_nano_cross_compile_objcopy",
        "objdump": "wrapper/k8_jetson_nano_cross_compile_objdump",
        "strip": "wrapper/k8_jetson_nano_cross_compile_strip",
    },
    toolchain_identifier = "gcc-k8-jetson_nano-cross-compile",
)

toolchain(
    name = "cc_toolchain_for_k8_jetson_nano_cross_compile",
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
        ":glibc_2_27",
    ],
    toolchain = ":cc_toolchain_k8_jetson_nano_cross_compile",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

toolchain(
    name = "cc_toolchain_for_k8",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    toolchain = ":cc_toolchain_linux_k8_gcc",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

toolchain(
    name = "cc_toolchain_for_aarch64",
    exec_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    target_compatible_with = [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    toolchain = ":cc_toolchain_linux_aarch64_gcc",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)
