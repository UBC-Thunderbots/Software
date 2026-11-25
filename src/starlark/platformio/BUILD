load("@rules_python//python:pip.bzl", "compile_pip_requirements")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

package(default_visibility = ["//visibility:public"])

compile_pip_requirements(
    name = "requirements",
    src = "requirements.in",
    # PlatformIO depends on chardet only on macOS/arm64
    # pip-compile would still write a platform-dependent `# via` comment for it.
    # Disable annotations so macOS and Linux generate identical lock files.
    extra_args = ["--no-annotate"],
    requirements_txt = "requirements_lock.txt",
)

# A hermetic PlatformIO binary, built from the pinned @platformio_deps pip hub
# rather than being resolved from the host PATH.
py_console_script_binary(
    name = "platformio",
    pkg = "@platformio_deps//platformio",
)
