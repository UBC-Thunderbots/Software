def _cross_compile_transition_impl(settings, attr):
    # This forces Bazel to switch the target platform for anything under this transition
    return {
        "//command_line_option:platforms": ["//toolchains/cc:robot"],
    }

_cross_compile_transition = transition(
    implementation = _cross_compile_transition_impl,
    inputs = [],
    outputs = ["//command_line_option:platforms"],
)

def _cross_compiled_filegroup_impl(ctx):
    # Collect the files generated under the new platform configuration
    # and pass them through as default outputs.
    return [DefaultInfo(files = depset(ctx.files.deps))]

cross_compiled_filegroup = rule(
    implementation = _cross_compiled_filegroup_impl,
    attrs = {
        "deps": attr.label_list(
            cfg = _cross_compile_transition,
            allow_files = True,
        ),
        # Mandatory attribute required by Bazel whenever using Starlark transitions
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
    },
)
