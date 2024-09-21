package(default_visibility = ["//visibility:public"])

load(":cc_toolchain_config.bzl", "cc_toolchain_config_jetson_nano", "cc_toolchain_config_k8")

platform(
    name = "jetson_nano_platform",
    constraint_values = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
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
    name = "linux_arm64_gcc_all",
    srcs = [
        ":linux_gcc_wrapper",
        "@linux_arm64_gcc//:includes",
        "@linux_arm64_gcc//:libs",
        "@linux_arm64_gcc//:runtime_libs",
        "@linux_arm64_gcc//:static_libs",
    ],
)

cc_toolchain_config_k8(
    name = "linux_k8_gcc",
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
    toolchain_identifier = "linux_k8_gcc-k8",
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
    toolchain_identifier = "k8_gcc-x86_64",
)

cc_toolchain(
    name = "cc_toolchain_linux_arm64_gcc",
    all_files = ":linux_arm64_gcc_all",
    ar_files = ":linux_arm64_gcc_all",
    as_files = ":linux_arm64_gcc_all",
    compiler_files = ":linux_arm64_gcc_all",
    dwp_files = ":linux_arm64_gcc_all",
    dynamic_runtime_lib = "@linux_arm64_gcc//:runtime_libs",
    linker_files = ":linux_arm64_gcc_all",
    objcopy_files = ":linux_arm64_gcc_all",
    static_runtime_lib = "@linux_arm64_gcc//:static_libs",
    strip_files = ":linux_arm64_gcc_all",
    supports_param_files = True,
    toolchain_config = ":linux_arm64_gcc",
    toolchain_identifier = "arm64_gcc-x86_64",
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

toolchain(
    name = "cc_toolchain_for_jetson_nano_linux",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    target_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    toolchain = ":cc_toolchain_jetson_nano",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

toolchain(
    name = "cc_toolchain_for_k8_fullsystem",
    exec_compatible_with = [
        "@platforms//os:linux",
        "@platforms//cpu:x86_64",
    ],
    toolchain = ":cc_toolchain_linux_k8_gcc",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)

toolchain(
    name = "cc_toolchain_for_arm64_fullsystem",
    exec_compatible_with = [
        "@platforms//cpu:arm64",
        "@platforms//os:linux",
    ],
    toolchain = ":cc_toolchain_linux_arm64_gcc",
    toolchain_type = "@bazel_tools//tools/cpp:toolchain_type",
)
