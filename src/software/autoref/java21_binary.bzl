"""Executable wrapper that selects Java 21 for its Java binary dependency."""

def _java21_transition_impl(settings, attr):
    return {
        "//command_line_option:java_runtime_version": "remotejdk_21",
    }

_java21_transition = transition(
    implementation = _java21_transition_impl,
    inputs = [],
    outputs = ["//command_line_option:java_runtime_version"],
)

def _java21_binary_impl(ctx):
    binary = ctx.attr.binary[0]
    binary_info = binary[DefaultInfo]
    executable = binary_info.files_to_run.executable

    launcher = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(
        output = launcher,
        target_file = executable,
        is_executable = True,
    )

    return [
        DefaultInfo(
            executable = launcher,
            runfiles = ctx.runfiles(
                files = [executable],
            ).merge(binary_info.default_runfiles),
        ),
    ]

java21_binary = rule(
    implementation = _java21_binary_impl,
    executable = True,
    attrs = {
        "binary": attr.label(
            mandatory = True,
            cfg = _java21_transition,
        ),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)