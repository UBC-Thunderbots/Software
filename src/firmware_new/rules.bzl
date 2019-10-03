load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain")

# TODO: Do we need this?
MyCCompileInfo = provider(doc = "", fields = ["object"])

# TODO: comment here
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
STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL = \
    "http://www.ebuc23.com/s3/stm_test/software/firmware/stm32cube_fw_h7_v150.zip"

# TODO: comment here
def _filter_none(input_list):
    filtered_list = []
    for element in input_list:
        if element != None:
            filtered_list.append(element)
    return filtered_list

""" Rule to Build A HAL Library With Specific Configuration """

def _cc_stm32h7_hal_library_impl(ctx):
    # Begin by copying all the requested files into the current directory
    # TODO: better comment here

    # TODO: rename "fw" to firmware..........

    fw_zip_name = "fw.zip"
    fw_zip = ctx.actions.declare_file(fw_zip_name)
    ctx.actions.run_shell(
        outputs = [fw_zip],
        command = """
        curl -L {} -o {}
        """.format(STM32_H7_DRIVERS_AND_MIDDLEWARE_DOWNLOAD_URL, fw_zip.path),
        progress_message = "Downloading firmware",
    )

    # TODO: the whole "external_deps" thing is a _bit_ convoluted, we can maybe simplify things a bit?
    drivers_and_middleware_hdrs = [
        ctx.actions.declare_file("external_deps/{}".format(filename))
        for filename in ctx.attr.drivers_and_middleware_hdrs
    ]
    drivers_and_middleware_srcs = [
        ctx.actions.declare_file("external_deps/{}".format(filename))
        for filename in ctx.attr.drivers_and_middleware_srcs
    ]

    external_deps_folder = ctx.actions.declare_directory("external_deps")
    print(external_deps_folder.path)
    ctx.actions.run_shell(
        inputs = [fw_zip],
        outputs = [external_deps_folder] + drivers_and_middleware_hdrs + drivers_and_middleware_srcs,
        command = """
        unzip -qo {} 'STM32Cube_FW_H7_V1.5.0/Drivers/*' 'STM32Cube_FW_H7_V1.5.0/Middlewares/*'
        rm -rf {external_deps_path}
        mv STM32Cube_FW_H7_V1.5.0 {external_deps_path}
        """.format(fw_zip.path, external_deps_path = external_deps_folder.path),
        progress_message = "Unzipping firmware",
    )

    #    ctx.actions.run_shell(
    #        outputs =
    #            drivers_and_middleware_hdrs +
    #            drivers_and_middleware_srcs +
    #            [external_deps_folder],
    #        arguments = ctx.attr.drivers_and_middleware_srcs + ctx.attr.drivers_and_middleware_hdrs,
    #        command = """
    #        external_deps_folder='%s'
    #        mkdir -p $external_deps_folder
    #        for file in "$@"
    #        do
    #            echo $external_deps_folder/$file
    #            mkdir -p $(dirname $external_deps_folder/$file)
    #            cp ~/STM32Cube/Repository/STM32Cube_FW_H7_V1.5.0/$file $external_deps_folder/$file
    #        done
    #        """ % external_deps_folder.path,
    #        # TODO: better progress message
    #        progress_message = "Getting requested driver and middleware files....",
    #    )

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )
    compilation_contexts = []
    linking_contexts = []

    print("drivers_and_middleware_srcs:{}".format(drivers_and_middleware_srcs))

    hal_config_hdr_includes = [file.dirname for file in ctx.files.hal_config_hdrs]
    drivers_and_middleware_includes = [file.dirname for file in drivers_and_middleware_hdrs]

    (compilation_context, compilation_outputs) = cc_common.compile(
        name = ctx.label.name,
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        public_hdrs = drivers_and_middleware_hdrs + ctx.files.hal_config_hdrs,
        srcs = drivers_and_middleware_srcs,
        includes = drivers_and_middleware_includes + hal_config_hdr_includes,
        user_compile_flags = ["-D{}".format(ctx.attr.processor), "-DUSE_HAL_DRIVER"],
        compilation_contexts = compilation_contexts,
    )
    print("compilation_outputs.objects:{}".format(compilation_outputs.objects))
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
        "drivers_and_middleware_hdrs": attr.string_list(),
        "drivers_and_middleware_srcs": attr.string_list(),
        "processor": attr.string(mandatory = True, values = SUPPORTED_PROCESSORS),
        "_cc_toolchain": attr.label(default = "@bazel_tools//tools/cpp:current_cc_toolchain"),
    },
    fragments = ["cpp"],
)

""" Rule to build a binary for the stm32h7 series of MCU's """
#def _cc_stm32_h7_binary_impl(ctx):
#    name = ctx.label.name
#    hal_config_files = ctx.files.hal_config_files
#    srcs = ctx.files.srcs
#    deps = ctx.attr.deps
#    cc_toolchain = find_cpp_toolchain(ctx)
#
#    hal_driver_dir = actions.declare_directory("hal_drivers")
#
#    # First build a HAL library configured specifically for this executable. We do this
#    # because each project has it's own "_hal_conf" files that result in compile-time
#    # changes to the library
#    hal_library_name = "{}_hal".format(name)
#    native.cc_library(
#        name = hal_library_name,
#        srcs = ["//firmware_new/Drivers/STM32H7xx_HAL_Driver:STM32H7xx_HAL_Driver_Srcs"],
#        hdrs = ["//firmware_new/Drivers/STM32H7xx_HAL_Driver:STM32H7xx_HAL_Driver_Hdrs"] + hal_config_files,
#        includes = ["../../Drivers/STM32H7xx_HAL_Driver/Inc", "./"],
#    )
#
#    # Build the `.elf` file
#    native.cc_binary(
#        name = "{}.elf".format(name),
#        srcs = srcs,
#        deps = [hal_library_name] + deps,
#    )
#
#    # TODO: comment explaining what this does and why it's here
#    native.genrule(
#        name = name,
#        srcs = ["{}.elf".format(name)],
#        outs = ["{}.bin".format(name)],
#        tools = ["@com_arm_developer_gcc//:objcopy"],
#        cmd = "$(location @com_arm_developer_gcc//:objcopy) -O binary $< $@",
#        output_to_bindir = True,
#    )
#
#    return [
#        DefaultInfo(files = depset(items = ["{}.bin".format(name)])),
#        MyCCompileInfo(object = "{}.bin".format(name)),
#    ]
#
#cc_stm32_h7_binary = rule(
#    implementation = _cc_stm32_h7_binary_impl,
#    attrs = {
#        "srcs": attr.label_list(mandatory = True, allow_files = True),
#        "deps": attr.label_list(allow_files = True),
#        "hal_config_files": attr.label_list(mandatory = True, allow_files = True),
#        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
#        # TODO: try this out
#        #        "_compiler": attr.label(
#        #                    default = Label("//tools:metalc"),
#        #                    allow_single_file = True,
#        #                    executable = True,
#        #                ),
#    },
#    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
#    fragments = ["cpp"],
#)
