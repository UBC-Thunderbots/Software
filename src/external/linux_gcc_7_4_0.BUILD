# -*- python -*-
# Adapted for linux gcc

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
    name = "libs",
    srcs = glob(["usr/lib/gcc/x86_64-linux-gnu/7.4.0/*.a"]),
)

filegroup(
    name = "includes",
    srcs = glob([
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/include/**",
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/include",
    ]),
)

filegroup(
    name = "runtime_libs",
    srcs = [
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/libstdc++.so",
    ],
)

filegroup(
    name = "static_libs",
    srcs = [
        "usr/lib/gcc/x86_64-linux-gnu/7.4.0/libstdc++.a",
    ],
)
