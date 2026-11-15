"""Reusable config_settings for selecting platform-specific binaries."""

# Maps a platform name to the @platforms constraints that identify it.
SUPPORTED_PLATFORMS = {
    "linux_amd64": [
        "@platforms//cpu:x86_64",
        "@platforms//os:linux",
    ],
    "linux_arm64": [
        "@platforms//cpu:aarch64",
        "@platforms//os:linux",
    ],
    "macos_arm64": [
        "@platforms//cpu:aarch64",
        "@platforms//os:macos",
    ],
}

def platform_config_settings():
    """Defines a `config_setting` for each supported platform.

    The generated settings are named after the platform, e.g. `:linux_amd64`
    and `:macos_arm64`, so they can be used directly as keys in a `select()`.
    """
    for platform, constraints in SUPPORTED_PLATFORMS.items():
        native.config_setting(
            name = platform,
            constraint_values = constraints,
            visibility = ["//visibility:public"],
        )
