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

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "binaries",
    srcs = [
        "bin/clang",
        "bin/clang++",
        "bin/llvm-dwp",
        "bin/llvm-objcopy",
        "bin/llvm-objdump",
        "bin/ld.lld",
        "bin/llvm-ar",
        "bin/llvm-nm",
        "bin/llvm-cov",
    ],
)

filegroup(
    name = "clang_libs",
    srcs = glob(["lib/clang/9.0.0/lib/linux/*.a"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "include/c++/**",
        "lib/clang/9.0.0/include/**",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = [
        "lib/libc++.so.1",
        "lib/libc++abi.so.1",
        "lib/libunwind.so.1",
    ],
)

filegroup(
    name = "static_libs",
    srcs = [
        "lib/libc++.a",
        "lib/libc++abi.a",
        "lib/libunwind.a",
    ],
)
