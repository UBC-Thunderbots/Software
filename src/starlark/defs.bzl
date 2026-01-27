load("@rules_pkg//pkg:providers.bzl", "PackageFilegroupInfo", "PackageFilesInfo", "PackageSymlinkInfo")

# ===================================================================
# Custom Build Rules and Definitions
# ===================================================================

# For maintaining Bazel runfile tree structure during packaging. Used in the onboard python CLI
# See for reference https://gist.github.com/pauldraper/7bc811ffbef6d3f3d4a4bb01afa9808f
def _runfile_path(workspace_name, file):
    path = file.short_path
    return path[len("../"):] if path.startswith("../") else "%s/%s" % (workspace_name, path)

def _runfiles_pkg_files(workspace_name, runfiles):
    files = {}
    for file in runfiles.files.to_list():
        files[_runfile_path(workspace_name, file)] = file
    for file in runfiles.symlinks.to_list():
        files[file.path] = "%s/%s" % (workspace_name, files.target_file)
    for file in runfiles.root_symlinks.to_list():
        files[file.path] = files.target_file

    return PackageFilesInfo(
        dest_src_map = files,
        attributes = {"mode": "0755"},
    )

def _pkg_runfiles_impl(ctx):
    runfiles = ctx.attr.runfiles[DefaultInfo]
    label = ctx.label
    workspace_name = ctx.workspace_name

    runfiles_files = _runfiles_pkg_files(workspace_name, runfiles.default_runfiles)

    pkg_filegroup_info = PackageFilegroupInfo(
        pkg_dirs = [],
        pkg_files = [(runfiles_files, label)],
        pkg_symlinks = [],
    )

    default_info = DefaultInfo(files = depset(runfiles_files.dest_src_map.values()))

    return [default_info, pkg_filegroup_info]

pkg_runfiles = rule(
    implementation = _pkg_runfiles_impl,
    attrs = {
        "runfiles": attr.label(
            doc = "Runfiles.",
            mandatory = True,
        ),
    },
    provides = [PackageFilegroupInfo],
)

def _pkg_executable_impl(ctx):
    bin = ctx.attr.bin[DefaultInfo]
    bin_executable = ctx.executable.bin
    path = ctx.attr.path
    label = ctx.label
    workspace_name = ctx.workspace_name

    runfiles_files = _runfiles_pkg_files(workspace_name, bin.default_runfiles)
    runfiles_files = PackageFilesInfo(
        dest_src_map = {"%s.runfiles/%s" % (path, p): file for p, file in runfiles_files.dest_src_map.items()},
        attributes = runfiles_files.attributes,
    )

    executable_symlink = PackageSymlinkInfo(
        attributes = {"mode": "0755"},
        destination = path,
        target = "%s.runfiles/%s" % (path, _runfile_path(workspace_name, bin_executable)),
    )

    pkg_filegroup_info = PackageFilegroupInfo(
        pkg_dirs = [],
        pkg_files = [(runfiles_files, label)],
        pkg_symlinks = [(executable_symlink, label)],
    )

    default_info = DefaultInfo(files = depset(runfiles_files.dest_src_map.values()))

    return [default_info, pkg_filegroup_info]

pkg_executable = rule(
    implementation = _pkg_executable_impl,
    attrs = {
        "bin": attr.label(
            doc = "Executable.",
            executable = True,
            cfg = "target",
            mandatory = True,
        ),
        "path": attr.string(
            doc = "Packaged path of executable. (Runfiles tree will be at <path>.runfiles.)",
            mandatory = True,
        ),
    },
    provides = [PackageFilegroupInfo],
)
