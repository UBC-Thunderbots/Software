MyCCompileInfo = provider(doc = "", fields = ["object"])

""" Rule to build a binary for the stm32h7 series of MCU's """

#def _cc_stm32_h7_binary_impl(ctx):
#    name = ctx.label.name
#    hal_config_files = ctx.files.hal_config_files
#    srcs = ctx.files.srcs
#    deps = ctx.attr.deps
#    cc_toolchain = find_cpp_toolchain(ctx)
def cc_stm32_h7_binary(**kwargs):
    name = kwargs.pop("name")
    hal_config_files = kwargs.pop("hal_config_files")
    srcs = kwargs.pop("srcs")
    deps = kwargs.pop("deps", [])

    # First build a HAL library configured specifically for this executable. We do this
    # because each project has it's own "_hal_conf" files that result in compile-time
    # changes to the library
    hal_library_name = "{}_hal".format(name)
    native.cc_library(
        name = hal_library_name,
        srcs = ["//firmware_new/Drivers/STM32H7xx_HAL_Driver:STM32H7xx_HAL_Driver_Srcs"],
        hdrs = ["//firmware_new/Drivers/STM32H7xx_HAL_Driver:STM32H7xx_HAL_Driver_Hdrs"] + hal_config_files,
        includes = ["../../Drivers/STM32H7xx_HAL_Driver/Inc", "./"],
    )

    # Build the `.elf` file
    native.cc_binary(
        name = "{}.elf".format(name),
        srcs = srcs,
        deps = [hal_library_name] + deps,
        **kwargs
    )

    # TODO: comment explaining what this does and why it's here
    native.genrule(
        name = name,
        srcs = ["{}.elf".format(name)],
        outs = ["{}.bin".format(name)],
        tools = ["@com_arm_developer_gcc//:objcopy"],
        cmd = "$(location @com_arm_developer_gcc//:objcopy) -O binary $< $@",
        output_to_bindir = True,
    )

    return [
        DefaultInfo(files = depset(items = ["{}.bin".format(name)])),
        MyCCompileInfo(object = "{}.bin".format(name)),
    ]

#cc_stm32_h7_binary = rule(
#    implementation = _cc_stm32_h7_binary_impl,
#    attrs = {
#        "srcs": attr.label_list(mandatory = True, allow_files = True),
#        "deps": attr.label_list(allow_files = True),
#        "hal_config_files": attr.label_list(mandatory = True, allow_files = True),
#        "_cc_toolchain": attr.label(default = Label("@bazel_tools//tools/cpp:current_cc_toolchain")),
#    },
#    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
#    fragments = ["cpp"],
#)
