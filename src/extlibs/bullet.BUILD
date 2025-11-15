"""
    Copyright 2020 Robert MacGregor

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

load("@rules_foreign_cc//foreign_cc:defs.bzl", "cmake")

filegroup(
    name = "bullet_files",
    srcs = glob(
        include = [
            "**/*",
        ],
    ),
)

cmake(
    name = "bullet_build",
    cache_entries = {
        "BUILD_BULLET3": "ON",
        "USE_MSVC_RUNTIME_LIBRARY_DLL": "ON",
        "BUILD_PYBULLET": "OFF",
        "BUILD_EXTRAS": "OFF",
        "BUILD_EGL": "OFF",
        # Force built libs to be installed
        "INSTALL_LIBS": "ON",
        "BUILD_UNIT_TESTS": "OFF",
        "BUILD_OPENGL3_DEMOS": "OFF",
        "BUILD_BULLET2_DEMOS": "OFF",
    },
    generate_args = select({
        "@bazel_tools//src/conditions:windows": ["-GNinja"],
        "//conditions:default": None,
    }),
    generate_crosstool_file = select({
        "@bazel_tools//src/conditions:windows": True,
        "//conditions:default": False,
    }),
    lib_source = ":bullet_files",
    out_static_libs = select({
        "@bazel_tools//src/conditions:windows": [
            "BulletDynamics.lib",
            "BulletCollision.lib",
            "Bullet3Collision.lib",
            "LinearMath.lib",
            "Bullet3Common.lib",
        ],

        # Linux
        "//conditions:default": [
            "libBulletDynamics.a",
            "libBulletCollision.a",
            "libBullet3Collision.a",
            "libLinearMath.a",
            "libBullet3Common.a",
        ],
    }),
)

# NOTE: This is necessary because of the way the bullet includes work
cc_library(
    name = "bullet",
    includes = [
        "bullet_build/include/bullet",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":bullet_build",
    ],
)
