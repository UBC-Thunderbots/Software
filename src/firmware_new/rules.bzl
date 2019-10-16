load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")

SUPPORTED_PROCESSORS = [
    "STM32H743xx",
    "STM32H753xx",
    "STM32H750xx",
    "STM32H742xx",
    "STM32H745xx",
    "STM32H755xx",
    "STM32H747xx",
    "STM32H757xx",
]

# We got this URL by running wireshark and watching what STM32CubeMX tried to download
# when it does its code generation
STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL = \
    "http://www.ebuc23.com/s3/stm_test/software/firmware/stm32cube_fw_h7_v150.zip"
STM32_H7_DRIVERS_AND_MIDDLEWARE_ZIP_PREFIX = "STM32Cube_FW_H7_V1.5.0"

def _filter_none(input_list):
    """ Returns the given list with all 'None' Elements Removed """
    filtered_list = []
    for element in input_list:
        if element != None:
            filtered_list.append(element)
    return filtered_list

def _cc_stm32h7_hal_library_impl(ctx):
    """
    Rule to build a HAL library for a stm32h7 MCU based on given configuration files
    """

    # Get the files we need to download in order to build the library
    drivers_and_middleware_hdrs = [
        ctx.actions.declare_file("external_deps/{}".format(filename))
        for filename in ctx.attr.drivers_and_middleware_hdrs
    ]
    drivers_and_middleware_srcs = [
        ctx.actions.declare_file("external_deps/{}".format(filename))
        for filename in ctx.attr.drivers_and_middleware_srcs
    ]

    # We assume that every header is included directly, ie. `#include "stm32h7xx.h"`
    # instead of `#include "Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h"`
    drivers_and_middleware_includes = [file.dirname for file in drivers_and_middleware_hdrs]

    # HAL is configured by having the user compile in headers with various options set
    hal_config_hdr_includes = [file.dirname for file in ctx.files.hal_config_hdrs]

    # Begin by downloading all third party files we need
    drivers_and_middleware_zip_name = "drivers_and_middleware.zip"
    drivers_and_middleware_zip = ctx.actions.declare_file(drivers_and_middleware_zip_name)
    ctx.actions.run_shell(
        outputs = [drivers_and_middleware_zip],
        command = """
        curl -L {download_url} -o {output_path}
        """.format(
            download_url = STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL,
            output_path = drivers_and_middleware_zip.path,
        ),
        progress_message = "Downloading drivers and middleware from {}".format(
            STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL,
        ),
    )

    # Unzip the files we just downloaded
    external_deps_folder = ctx.actions.declare_directory("external_deps")
    ctx.actions.run_shell(
        inputs = [drivers_and_middleware_zip],
        outputs =
            [external_deps_folder] +
            drivers_and_middleware_hdrs +
            drivers_and_middleware_srcs,
        command = """
        unzip -qo {path_to_zip} '{zip_prefix}/Drivers/*' '{zip_prefix}/Middlewares/*'
        rm -rf {external_deps_path}
        mv {zip_prefix} {external_deps_path}
        """.format(
            path_to_zip = drivers_and_middleware_zip.path,
            zip_prefix = STM32_H7_DRIVERS_AND_MIDDLEWARE_ZIP_PREFIX,
            external_deps_path = external_deps_folder.path,
        ),
        progress_message = "Unzipping drivers and middleware",
    )

    # Setup the toolchain
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    compilation_contexts = []
    linking_contexts = []

    # Compile
    (compilation_context, compilation_outputs) = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        public_hdrs =
            drivers_and_middleware_hdrs +
            ctx.files.hal_config_hdrs +
            ctx.files.free_rtos_config_hdrs,
        srcs = drivers_and_middleware_srcs,
        includes =
            drivers_and_middleware_includes +
            hal_config_hdr_includes,
        user_compile_flags = ["-D{}".format(ctx.attr.processor), "-DUSE_HAL_DRIVER"],
        compilation_contexts = compilation_contexts,
    )

    # Link
    (linking_context, linking_outputs) = cc_common.create_linking_context_from_compilation_outputs(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        language = "c++",
        compilation_outputs = compilation_outputs,
        linking_contexts = linking_contexts,
    )
    library = linking_outputs.library_to_link
    files = []
    files.extend(compilation_outputs.objects)
    files.extend(compilation_outputs.pic_objects)
    files.append(library.pic_static_library)
    files.append(library.static_library)
    files.append(library.dynamic_library)

    return [
        DefaultInfo(
            files = depset(_filter_none(files)),
        ),
        CcInfo(
            compilation_context = compilation_context,
            linking_context = linking_context,
        ),
    ]

cc_stm32h7_hal_library = rule(
    implementation = _cc_stm32h7_hal_library_impl,
    attrs = {
        "hal_config_hdrs": attr.label_list(allow_files = [".h"]),
        "free_rtos_config_hdrs": attr.label_list(allow_files = [".h"]),
        "drivers_and_middleware_hdrs": attr.string_list(),
        "drivers_and_middleware_srcs": attr.string_list(),
        "processor": attr.string(mandatory = True, values = SUPPORTED_PROCESSORS),
        #        "_cc_toolchain": attr.label(default = "@bazel_tools//tools/cpp:current_cc_toolchain"),
        "_cc_toolchain": attr.label(default = "//tools/cc_toolchain:stm32h7_toolchain"),
    },
    fragments = ["cpp"],
)

def _cc_stm32h7_binary_impl(ctx):
    """
    Rule to build binaries (.bin and .elf) for a stm32h7 MCU
    """

    # Initial configuration
    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    # Add all dependencies for this target to the required contexts for compilation
    # and linking
    compilation_contexts = []
    linking_contexts = []
    for dep in ctx.attr.deps:
        if CcInfo in dep:
            compilation_contexts.append(dep[CcInfo].compilation_context)
            linking_contexts.append(dep[CcInfo].linking_context)

    # Manually compile the assembly files
    assembly_object_files = []
    for assembly_file in ctx.files.assembly:
        object_file = ctx.actions.declare_file("_objs/{}.o".format(assembly_file.basename))
        ctx.actions.run_shell(
            inputs = [assembly_file],
            outputs = [object_file],
            tools = cc_toolchain.all_files,
            command = "{gcc_bin} -c -x assembler-with-cpp {assembly_file} -o {obj_out} ".format(
                gcc_bin = cc_toolchain.compiler_executable(),
                obj_out = object_file.path,
                assembly_file = assembly_file.path,
            ),
        )
        assembly_object_files.append(object_file)
    assembly_compilation_outputs = cc_common.create_compilation_outputs(
        objects = depset(_filter_none(assembly_object_files)),
        pic_objects = depset(_filter_none(assembly_object_files)),
    )

    # Compile the source code normally
    (_compilation_context, compilation_outputs) = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        srcs = ctx.files.srcs,
        private_hdrs = ctx.files.hdrs,
        compilation_contexts = compilation_contexts,
        user_compile_flags = ctx.attr.copts + ["-D{}".format(ctx.attr.processor), "-DUSE_HAL_DRIVER"],
    )

    # Link
    linking_outputs = cc_common.link(
        name = "{}.elf".format(ctx.label.name),
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        language = "c++",
        compilation_outputs = cc_common.merge_compilation_outputs(
            compilation_outputs = [assembly_compilation_outputs, compilation_outputs],
        ),
        linking_contexts = linking_contexts,
        user_link_flags = ctx.attr.linkopts + [
            "-T{}".format(ctx.file.linker_script.path),
            "-specs=nano.specs",
            "-lc",
            "-lm",
            "-lnosys",
        ],
        link_deps_statically = True,
        additional_inputs = [ctx.file.linker_script],
        output_type = "executable",
    )

    # Create the .bin from the .elf
    elf_file = linking_outputs.executable
    bin_file = ctx.actions.declare_file("{}.bin".format(ctx.label.name))
    ctx.actions.run_shell(
        inputs = [elf_file],
        outputs = [bin_file],
        tools = cc_toolchain.all_files,
        command = "{objcopy} -O binary {elf_out} {cc_bin}".format(
            objcopy = cc_toolchain.objcopy_executable(),
            elf_out = elf_file.path,
            cc_bin = bin_file.path,
        ),
    )

    return [
        DefaultInfo(
            files = depset(_filter_none([elf_file, bin_file])),
        ),
    ]

cc_stm32h7_binary = rule(
    implementation = _cc_stm32h7_binary_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = [".c"]),
        "assembly": attr.label_list(allow_files = [".s"]),
        # TODO: Should not allow headers here?
        "hdrs": attr.label_list(allow_files = [".h"]),
        "linker_script": attr.label(mandatory = True, allow_single_file = True),
        "deps": attr.label_list(allow_empty = True, providers = [CcInfo]),
        "data": attr.label_list(default = [], allow_files = True),
        "linkopts": attr.string_list(),
        "copts": attr.string_list(),
        "_cc_toolchain": attr.label(default = "//tools/cc_toolchain:stm32h7_toolchain"),
        "processor": attr.string(mandatory = True, values = SUPPORTED_PROCESSORS),
    },
    fragments = ["cpp"],
)
