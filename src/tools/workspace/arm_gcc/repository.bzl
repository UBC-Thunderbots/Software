# -*- python -*-

# Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def arm_gcc_repository(name):
    http_archive(
        name = name,
        urls = [
            "https://developer.arm.com/-/media/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2",
        ],
        sha256 = "b50b02b0a16e5aad8620e9d7c31110ef285c1dde28980b1a9448b764d77d8f92",
        strip_prefix = "gcc-arm-none-eabi-8-2019-q3-update",
        build_file = Label("//tools/workspace/arm_gcc:package.BUILD"),
    )
