load("@bazel_lib//lib:copy_file.bzl", "copy_file")
load("@rules_python//python:defs.bzl", "py_library")
load("@thunderscope_deps//:requirements.bzl", "requirement")
load("//starlark:platforms.bzl", "platform_config_settings")

package(default_visibility = ["//visibility:public"])

platform_config_settings()

copy_file(
    name = "gamecontroller_bin",
    src = select({
        ":linux_amd64": "@ssl_game_controller_linux_amd64//file",
        ":linux_arm64": "@ssl_game_controller_linux_arm64//file",
        ":macos_arm64": "@ssl_game_controller_darwin_arm64//file",
    }),
    out = "gamecontroller",
    allow_symlink = True,
    is_executable = True,
)

py_library(
    name = "simulator",
    srcs = [
        "simulator.py",
        "util.py",
    ],
    data = [
        "//software:er_force_simulator_main",
    ],
    deps = [
        requirement("psutil"),
    ],
)

py_library(
    name = "game_controller",
    srcs = [
        "game_controller.py",
        "util.py",
    ],
    data = [
        ":gamecontroller_bin",
    ],
    deps = [
        "//proto:import_all_protos",
        "//software/networking:ssl_proto_communication",
        "//software/thunderscope:field_frame",
        "//software/thunderscope:util",
        "//software/thunderscope/common:thread_safe_circular_buffer",
        "@rules_python//python/runfiles",
        requirement("netifaces"),
        requirement("psutil"),
    ],
)

py_library(
    name = "full_system",
    srcs = [
        "full_system.py",
        "util.py",
    ],
    data = [
        "//software:unix_full_system",
    ],
    deps = [
        requirement("psutil"),
    ],
)

py_library(
    name = "tigers_autoref",
    srcs = [
        "tigers_autoref.py",
    ],
    data = [
        "//software/autoref:autoref_java",
        "@tigers_autoref//:config",
    ],
    deps = [
        ":game_controller",
        "//software/thunderscope:time_provider",
        "@rules_python//python/runfiles",
    ],
)

py_library(
    name = "runtime_manager",
    srcs = [
        "runtime_installer.py",
        "runtime_loader.py",
        "runtime_manager.py",
    ],
    deps = [
        requirement("requests"),
    ],
)
