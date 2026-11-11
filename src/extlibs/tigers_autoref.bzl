# Description:
#   TIGERs AutoReferee is an automatic referee for RoboCup Small Size League.
#   https://github.com/TIGERs-Mannheim/AutoReferee
#
#   This BUILD overlay exposes the prebuilt distribution downloaded by
#   http_archive in MODULE.bazel. 
#
#   This target provides runtime files only. Starting AutoRef with a
#   Bazel-managed Java runtime is handled separately.

AUTOREF_RUNTIME_FILES = [
    # Upstream launch scripts.
    "bin/**",
    # Configuration files used by AutoRef at runtime.
    "config/**",
    # Application and dependency JARs used by the upstream launcher.
    "lib/**",
]

# Keep the complete distribution available to consumers so the launcher's
# expected bin/, config/, and lib/ directory layout can be preserved.
filegroup(
    name = "distribution",
    srcs = glob(AUTOREF_RUNTIME_FILES),
    visibility = ["//visibility:public"],
)