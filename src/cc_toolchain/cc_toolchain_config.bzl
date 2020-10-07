# -*- python -*-
# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This was originally taken from https://github.com/mjbots/rules_mbed, with many
# thunderbots-specific modifications

load(
    "@bazel_tools//tools/cpp:cc_toolchain_config_lib.bzl",
    "action_config",
    "env_entry",
    "env_set",
    "feature",
    "feature_set",
    "flag_group",
    "flag_set",
    "tool",
    "tool_path",
    "variable_with_value",
)
load(
    "@bazel_tools//tools/build_defs/cc:action_names.bzl",
    "ASSEMBLE_ACTION_NAME",
    "CC_FLAGS_MAKE_VARIABLE_ACTION_NAME",
    "CLIF_MATCH_ACTION_NAME",
    "CPP_COMPILE_ACTION_NAME",
    "CPP_HEADER_PARSING_ACTION_NAME",
    "CPP_LINK_DYNAMIC_LIBRARY_ACTION_NAME",
    "CPP_LINK_EXECUTABLE_ACTION_NAME",
    "CPP_LINK_NODEPS_DYNAMIC_LIBRARY_ACTION_NAME",
    "CPP_LINK_STATIC_LIBRARY_ACTION_NAME",
    "CPP_MODULE_CODEGEN_ACTION_NAME",
    "CPP_MODULE_COMPILE_ACTION_NAME",
    "C_COMPILE_ACTION_NAME",
    "LINKSTAMP_COMPILE_ACTION_NAME",
    "LTO_BACKEND_ACTION_NAME",
    "LTO_INDEXING_ACTION_NAME",
    "PREPROCESS_ASSEMBLE_ACTION_NAME",
    "STRIP_ACTION_NAME",
)

ACTION_NAMES = struct(
    c_compile = C_COMPILE_ACTION_NAME,
    cpp_compile = CPP_COMPILE_ACTION_NAME,
    linkstamp_compile = LINKSTAMP_COMPILE_ACTION_NAME,
    cc_flags_make_variable = CC_FLAGS_MAKE_VARIABLE_ACTION_NAME,
    cpp_module_codegen = CPP_MODULE_CODEGEN_ACTION_NAME,
    cpp_header_parsing = CPP_HEADER_PARSING_ACTION_NAME,
    cpp_module_compile = CPP_MODULE_COMPILE_ACTION_NAME,
    assemble = ASSEMBLE_ACTION_NAME,
    preprocess_assemble = PREPROCESS_ASSEMBLE_ACTION_NAME,
    lto_indexing = LTO_INDEXING_ACTION_NAME,
    lto_backend = LTO_BACKEND_ACTION_NAME,
    cpp_link_executable = CPP_LINK_EXECUTABLE_ACTION_NAME,
    cpp_link_dynamic_library = CPP_LINK_DYNAMIC_LIBRARY_ACTION_NAME,
    cpp_link_nodeps_dynamic_library = CPP_LINK_NODEPS_DYNAMIC_LIBRARY_ACTION_NAME,
    cpp_link_static_library = CPP_LINK_STATIC_LIBRARY_ACTION_NAME,
    strip = STRIP_ACTION_NAME,
    clif_match = CLIF_MATCH_ACTION_NAME,
    objcopy_embed_data = "objcopy_embed_data",
    ld_embed_data = "ld_embed_data",
)

ALL_COMPILE_ACTIONS = [
    ACTION_NAMES.c_compile,
    ACTION_NAMES.cpp_compile,
    ACTION_NAMES.linkstamp_compile,
    ACTION_NAMES.cc_flags_make_variable,
    ACTION_NAMES.cpp_module_codegen,
    ACTION_NAMES.cpp_header_parsing,
    ACTION_NAMES.cpp_module_compile,
    ACTION_NAMES.assemble,
    ACTION_NAMES.preprocess_assemble,
    ACTION_NAMES.lto_indexing,
    ACTION_NAMES.lto_backend,
    ACTION_NAMES.strip,
    ACTION_NAMES.clif_match,
]

ALL_LINK_ACTIONS = [
    ACTION_NAMES.cpp_link_executable,
    ACTION_NAMES.cpp_link_dynamic_library,
    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
]

ALL_CPP_ACTIONS = [
    ACTION_NAMES.cpp_compile,
    ACTION_NAMES.cpp_module_codegen,
    ACTION_NAMES.cpp_header_parsing,
    ACTION_NAMES.cpp_module_compile,
    ACTION_NAMES.cpp_link_executable,
    ACTION_NAMES.cpp_link_dynamic_library,
    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
]

def _make_common_features(ctx):
    result = {}

    result["static_link_cpp_runtimes"] = feature(
        name = "static_link_cpp_runtimes",
    )

    result["unfiltered_compile_flags_feature"] = feature(
        name = "unfiltered_compile_flags",
        flag_sets = ([
            flag_set(
                actions = [
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                ],
                flag_groups = [
                    flag_group(
                        flags = ctx.attr.host_unfiltered_compile_flags,
                    ),
                ],
            ),
        ] if ctx.attr.host_unfiltered_compile_flags else []),
    )

    result["determinism_feature"] = feature(
        name = "determinism",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Wno-builtin-macro-redefined",
                            "-D__DATE__=\"redacted\"",
                            "-D__TIMESTAMP__=\"redacted\"",
                            "-D__TIME__=\"redacted\"",
                        ],
                    ),
                ],
            ),
        ],
    )

    result["hardening_feature"] = feature(
        name = "hardening",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-U_FORTIFY_SOURCE",
                            "-D_FORTIFY_SOURCE=1",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = [
                    ACTION_NAMES.cpp_link_dynamic_library,
                    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
                ],
                flag_groups = [flag_group(flags = ["-Wl,-z,relro,-z,now"])],
            ),
            flag_set(
                actions = [ACTION_NAMES.cpp_link_executable],
                flag_groups = [flag_group(flags = ["-Wl,-z,relro,-z,now"])],
            ),
        ],
    )

    result["supports_dynamic_linker_feature"] = feature(
        name = "supports_dynamic_linker",
        enabled = True,
    )

    result["warnings_as_errors_feature"] = feature(
        name = "warnings_as_errors",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(
                        # -Werror can be overridden by copts argument in cc_binary
                        # Warnings are added in .bazelrc
                        flags = ["-Werror"] +
                                ctx.attr.host_compiler_warnings,
                    ),
                ],
            ),
        ],
    )

    result["colour_feature"] = feature(
        name = "colour",
        # the compiler will highlight warnings and errors with colour
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(
                        flags = ["-fdiagnostics-color=always"],
                    ),
                ],
            ),
        ],
    )

    result["dbg_feature"] = feature(
        name = "dbg",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [flag_group(flags = [
                    "-O0",
                    "-g3",
                ])],
            ),
        ],
        implies = ["common"],
    )

    result["disable_assertions_feature"] = feature(
        name = "disable-assertions",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [flag_group(flags = ["-DNDEBUG"])],
            ),
        ],
    )

    result["fastbuild_feature"] = feature(name = "fastbuild", implies = ["common"])

    result["user_compile_flags_feature"] = feature(
        name = "user_compile_flags",
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.preprocess_assemble,
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                    ACTION_NAMES.cpp_header_parsing,
                    ACTION_NAMES.cpp_module_compile,
                    ACTION_NAMES.cpp_module_codegen,
                ],
                flag_groups = [
                    flag_group(
                        flags = ["%{user_compile_flags}"],
                        iterate_over = "user_compile_flags",
                        expand_if_available = "user_compile_flags",
                    ),
                ],
            ),
        ],
    )

    result["frame_pointer_feature"] = feature(
        name = "frame-pointer",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [flag_group(flags = ["-fno-omit-frame-pointer"])],
            ),
        ],
    )

    result["build_id_feature"] = feature(
        name = "build-id",
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [
                    flag_group(
                        flags = ["-Wl,--build-id=md5", "-Wl,--hash-style=gnu"],
                    ),
                ],
            ),
        ],
    )

    result["no_stripping_feature"] = feature(name = "no_stripping")

    result["no_canonical_prefixes_feature"] = feature(
        name = "no-canonical-prefixes",
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-no-canonical-prefixes",
                        ] + ctx.attr.extra_no_canonical_prefixes_flags,
                    ),
                ],
            ),
        ],
    )

    result["no_canonical_system_headers"] = feature(
        name = "no-canonical-system-headers",
        flag_sets = [
            flag_set(
                actions = [
                    ACTION_NAMES.c_compile,
                    ACTION_NAMES.cpp_compile,
                ],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-fno-canonical-system-headers",
                        ],
                    ),
                ],
            ),
        ],
    )

    result["has_configured_linker_path_feature"] = feature(name = "has_configured_linker_path")

    result["copy_dynamic_libraries_to_binary_feature"] = \
        feature(name = "copy_dynamic_libraries_to_binary")

    result["user_link_flags_feature"] = feature(
        name = "user_link_flags",
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [
                    flag_group(
                        flags = ["%{user_link_flags}"],
                        iterate_over = "user_link_flags",
                        expand_if_available = "user_link_flags",
                    ),
                ],
            ),
        ],
    )

    result["cpp17_feature"] = feature(
        name = "c++17",
        flag_sets = [
            flag_set(
                actions = ALL_CPP_ACTIONS,
                flag_groups = [flag_group(flags = ["-std=c++17"])],
            ),
        ],
    )

    result["no-rtti"] = feature(
        name = "no-rtti",
        flag_sets = [
            flag_set(
                actions = ALL_CPP_ACTIONS,
                flag_groups = [flag_group(flags = ["-fno-rtti"])],
            ),
        ],
    )

    result["no-exceptions"] = feature(
        name = "no-exceptions",
        flag_sets = [
            flag_set(
                actions = ALL_CPP_ACTIONS,
                flag_groups = [flag_group(flags = ["-fno-exceptions"])],
            ),
        ],
    )

    return result

def _linux_gcc_impl(ctx):
    host_system_name = "k8"

    action_configs = []

    tool_paths = [
        tool_path(name = name, path = path)
        for name, path in ctx.attr.tool_paths.items()
    ]

    common = _make_common_features(ctx)

    runtime_library_search_directories = feature(
        name = "runtime_library_search_directories",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [
                    flag_group(
                        flags = [
                            "-Wl,-rpath,$ORIGIN/%{runtime_library_search_directories}",
                        ],
                        iterate_over = "runtime_library_search_directories",
                        expand_if_available = "runtime_library_search_directories",
                    ),
                ],
            ),
        ],
    )

    static_libgcc = feature(
        name = "static-libgcc",
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [
                    flag_group(flags = ["-static-libgcc"]),
                ],
            ),
        ],
    )

    pic_feature = feature(
        name = "pic",
        enabled = True,
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(flags = ["-fPIC"]),
                ],
            ),
        ],
    )

    supports_pic_feature = feature(name = "supports_pic", enabled = True)

    stdlib_feature = feature(
        name = "stdlib",
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [flag_group(flags = [
                    "-lm",
                    "-lpthread",
                    "-ldl",
                    "-lrt",
                    "-lstdc++fs",
                ])],
            ),
        ],
    )

    lld_feature = feature(
        name = "lld",
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [flag_group(flags = [
                    "-fuse-ld=gold",
                ])],
            ),
        ],
    )

    opt_feature = feature(
        name = "opt",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-ggdb",
                            "-O3",
                            "-ffunction-sections",
                            "-fdata-sections",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = [
                    ACTION_NAMES.cpp_link_dynamic_library,
                    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
                    ACTION_NAMES.cpp_link_executable,
                ],
                flag_groups = [flag_group(flags = ["-Wl,--gc-sections"])],
            ),
        ],
        implies = ["common"],
    )

    coverage_feature = feature(
        name = "coverage",
        flag_sets = [
            flag_set(
                actions = ALL_COMPILE_ACTIONS,
                flag_groups = [
                    flag_group(
                        flags = ["--coverage"],
                    ),
                ],
            ),
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [
                    flag_group(
                        flags = ["--coverage"],
                    ),
                ],
            ),
        ],
        provides = ["profile"],
    )

    # This feature is not required for the code to build, but it is required
    # in order for the bazel plugin we use for CLion to function correctly. This was
    # prompted by the following comment in the plugin itself:
    # https://github.com/bazelbuild/intellij/blob/e76fdadb0bdabbe2be913cba03d9014eb2366374/cpp/src/com/google/idea/blaze/cpp/SysrootFlagsProcessor.java#L70
    # An issue has been filed (https://github.com/bazelbuild/intellij/issues/1368) to
    # hopefully correct this so we don't need this feature in the future, or confirm
    # if this is expected behavior.
    builtin_include_directories_feature = feature(
        name = "builtin_include_directories",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [flag_group(
                    flags = [
                        "-I{}".format(dir)
                        for dir in ctx.attr.builtin_include_directories
                    ],
                )],
            ),
        ],
    )

    common_feature = feature(
        name = "common",
        implies = [
            "builtin_include_directories",
            "c++17",
            "colour",
            "determinism",
            "warnings_as_errors",
            "hardening",
            "build-id",
            "no-canonical-prefixes",
            "stdlib",
            "lld",
            "frame-pointer",
            "static_link_cpp_runtimes",
        ],
    )

    features = common.values() + [
        static_libgcc,
        pic_feature,
        supports_pic_feature,
        builtin_include_directories_feature,
        common_feature,
        stdlib_feature,
        lld_feature,
        coverage_feature,
        opt_feature,
        runtime_library_search_directories,
    ]

    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, "Fake executable")

    return [
        cc_common.create_cc_toolchain_config_info(
            ctx = ctx,
            features = features,
            action_configs = action_configs,
            artifact_name_patterns = [],
            cxx_builtin_include_directories = ctx.attr.builtin_include_directories,
            toolchain_identifier = ctx.attr.toolchain_identifier,
            host_system_name = host_system_name,
            target_system_name = ctx.attr.target_system_name,
            target_cpu = ctx.attr.target_cpu,
            target_libc = "libc",
            compiler = "gcc",
            abi_version = "local",
            abi_libc_version = "local",
            tool_paths = tool_paths,
            make_variables = [],
            builtin_sysroot = None,
            cc_target_os = None,
        ),
        DefaultInfo(
            executable = out,
        ),
    ]

cc_toolchain_config_k8 = rule(
    implementation = _linux_gcc_impl,
    attrs = {
        "builtin_include_directories": attr.string_list(),
        "cpu": attr.string(mandatory = True, values = ["k8"]),
        "extra_features": attr.string_list(),
        "extra_no_canonical_prefixes_flags": attr.string_list(),
        "host_compiler_warnings": attr.string_list(),
        "host_unfiltered_compile_flags": attr.string_list(),
        "target_cpu": attr.string(),
        "target_system_name": attr.string(),
        "tool_paths": attr.string_dict(),
        "toolchain_identifier": attr.string(),
    },
    provides = [CcToolchainConfigInfo],
    executable = True,
)

def _stm32_impl(ctx):
    host_system_name = "k8"

    action_configs = []

    common = _make_common_features(ctx)

    stdlib_feature = feature(
        name = "stdlib",
        flag_sets = [
            flag_set(
                actions = ALL_LINK_ACTIONS,
                flag_groups = [flag_group(flags = [
                    "-Wl,--start-group",
                    "-lstdc++",
                    "-lsupc++",
                    "-lm",
                    "-lc",
                    "-lgcc",
                    "-lnosys",
                    "-Wl,--end-group",
                ])],
            ),
        ],
    )

    opt_feature = feature(
        name = "opt",
        flag_sets = [
            flag_set(
                actions = [ACTION_NAMES.c_compile, ACTION_NAMES.cpp_compile],
                flag_groups = [
                    flag_group(
                        flags = [
                            "-g",
                            "-Os",
                            "-ffunction-sections",
                            "-fdata-sections",
                        ],
                    ),
                ],
            ),
            flag_set(
                actions = [
                    ACTION_NAMES.cpp_link_dynamic_library,
                    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
                    ACTION_NAMES.cpp_link_executable,
                ],
                flag_groups = [flag_group(flags = ["-Wl,--gc-sections"])],
            ),
        ],
        implies = ["common"],
    )

    stm32_feature = feature(
        name = "stm32",
        flag_sets = [
            flag_set(
                actions = ALL_COMPILE_ACTIONS + ALL_LINK_ACTIONS,
                flag_groups = [flag_group(flags = [
                    "-mthumb",
                    "-mfloat-abi=softfp",
                ])],
            ),
        ],
        implies = [
            "no-canonical-system-headers",
            "no-rtti",
            "no-exceptions",
        ],
    )

    stm32f4_feature = feature(
        name = "stm32f4",
        flag_sets = [
            flag_set(
                actions = ALL_COMPILE_ACTIONS + ALL_LINK_ACTIONS,
                flag_groups = [flag_group(flags = [
                    "-mcpu=cortex-m4",
                    "-mfpu=fpv4-sp-d16",
                ])],
            ),
        ],
        implies = ["stm32"],
    )

    stm32h7_feature = feature(
        name = "stm32h7",
        flag_sets = [
            flag_set(
                actions = ALL_COMPILE_ACTIONS + ALL_LINK_ACTIONS,
                flag_groups = [flag_group(flags = [
                    "-mcpu=cortex-m7",
                    "-mfpu=fpv5-d16",
                ])],
            ),
        ],
        implies = ["stm32"],
    )

    common_feature = feature(
        name = "common",
        implies = [
            "stdlib",
            "c++17",
            "colour",
            "warnings_as_errors",
            "determinism",
            "no-canonical-prefixes",
        ] + ([ctx.attr.cpu] if ctx.attr.cpu in [
            "stm32f4",
            "stm32h7",
        ] else []),
    )

    features = common.values() + [
        stdlib_feature,
        common_feature,
        opt_feature,
        stm32_feature,
        stm32f4_feature,
        stm32h7_feature,
    ]

    cxx_builtin_include_directories = ctx.attr.builtin_include_directories

    tool_paths = [
        tool_path(name = "gcc", path = ctx.attr.host_compiler_path),
        tool_path(name = "ar", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-ar"),
        tool_path(name = "compat-ld", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-ld"),
        tool_path(name = "cpp", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-cpp"),
        tool_path(name = "gcov", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-gcov"),
        tool_path(name = "ld", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-ld"),
        tool_path(name = "nm", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-nm"),
        tool_path(name = "objcopy", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-objcopy"),
        tool_path(name = "objdump", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-objdump"),
        tool_path(name = "strip", path = ctx.attr.host_compiler_prefix + "/arm-none-eabi-strip"),
    ]

    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, "Fake executable")

    return [
        cc_common.create_cc_toolchain_config_info(
            ctx = ctx,
            features = features,
            action_configs = action_configs,
            artifact_name_patterns = [],
            cxx_builtin_include_directories = cxx_builtin_include_directories,
            toolchain_identifier = ctx.attr.toolchain_identifier,
            host_system_name = host_system_name,
            target_system_name = ctx.attr.target_system_name,
            target_cpu = ctx.attr.target_cpu,
            target_libc = "libc",
            compiler = "clang",
            abi_version = "local",
            abi_libc_version = "local",
            tool_paths = tool_paths,
            make_variables = [],
            builtin_sysroot = None,
            cc_target_os = None,
        ),
        DefaultInfo(
            executable = out,
        ),
    ]

cc_toolchain_config_stm32 = rule(
    implementation = _stm32_impl,
    attrs = {
        "cpu": attr.string(mandatory = True, values = [
            "stm32f4",
            "stm32h7",
        ]),
        "builtin_include_directories": attr.string_list(),
        "extra_no_canonical_prefixes_flags": attr.string_list(),
        "host_compiler_path": attr.string(),
        "host_compiler_prefix": attr.string(),
        "host_compiler_warnings": attr.string_list(),
        "host_unfiltered_compile_flags": attr.string_list(),
        "target_cpu": attr.string(),
        "target_system_name": attr.string(),
        "toolchain_identifier": attr.string(),
        "extra_features": attr.string_list(),
    },
    provides = [CcToolchainConfigInfo],
    executable = True,
)
