# BUILD file for the Bullet Physics SDK (bullet3).
#
# Bullet is a CMake project with no upstream Bazel support, so this file
# mirrors the library layout in src/CMakeLists.txt: one cc_library per
# component, with dependencies derived from the components' #include graph.
#
# Bullet headers are included relative to "src" (for example
# "LinearMath/btVector3.h" or "BulletCollision/..."), so every target exposes
# "src" as an include directory; it propagates to consumers transitively.
#
# The defaults match Bullet's default CMake options: single precision (float)
# and no multithreading. To build with doubles instead, pass
# "--copt=-DBT_USE_DOUBLE_PRECISION" so that every translation unit (including
# yours) agrees on the ABI.
#
# Deliberately not covered (they need toolchains/tooling beyond the core libs):
#   * src/Bullet3OpenCL and src/clew  (require an OpenCL SDK)
#   * Extras/ and examples/           (graphics, serialization, robotics, ...)

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "LinearMath",
    srcs = glob(["src/LinearMath/**/*.cpp"]),
    hdrs = glob(["src/LinearMath/**/*.h"]),
    includes = ["src"],
)

cc_library(
    name = "BulletCollision",
    srcs = glob(["src/BulletCollision/**/*.cpp"]),
    hdrs = glob(["src/BulletCollision/**/*.h"]) + ["src/btBulletCollisionCommon.h"],
    includes = ["src"],
    deps = [":LinearMath"],
)

cc_library(
    name = "BulletDynamics",
    srcs = glob(["src/BulletDynamics/**/*.cpp"]),
    hdrs = glob(["src/BulletDynamics/**/*.h"]) + ["src/btBulletDynamicsCommon.h"],
    includes = ["src"],
    deps = [
        ":Bullet3Common",
        ":BulletCollision",
        ":LinearMath",
    ],
)

cc_library(
    name = "BulletSoftBody",
    srcs = glob(["src/BulletSoftBody/**/*.cpp"]),
    hdrs = glob(["src/BulletSoftBody/**/*.h"]),
    includes = ["src"],
    deps = [
        ":BulletCollision",
        ":BulletDynamics",
        ":LinearMath",
    ],
)

cc_library(
    name = "Bullet3Common",
    srcs = glob(["src/Bullet3Common/**/*.cpp"]),
    hdrs = glob(["src/Bullet3Common/**/*.h"]),
    includes = ["src"],
)

cc_library(
    name = "BulletInverseDynamics",
    srcs = glob(["src/BulletInverseDynamics/**/*.cpp"]),
    hdrs = glob(["src/BulletInverseDynamics/**/*.hpp"]),
    includes = ["src"],
    deps = [
        ":Bullet3Common",
        ":LinearMath",
    ],
)

cc_library(
    name = "Bullet3Geometry",
    srcs = glob(["src/Bullet3Geometry/**/*.cpp"]),
    hdrs = glob(["src/Bullet3Geometry/**/*.h"]),
    includes = ["src"],
    deps = [":Bullet3Common"],
)

cc_library(
    name = "Bullet3Collision",
    srcs = glob(["src/Bullet3Collision/**/*.cpp"]),
    hdrs = glob(["src/Bullet3Collision/**/*.h"]),
    includes = ["src"],
    deps = [
        ":Bullet3Common",
        ":Bullet3Geometry",
    ],
)

cc_library(
    name = "Bullet3Dynamics",
    srcs = glob(["src/Bullet3Dynamics/**/*.cpp"]),
    hdrs = glob(["src/Bullet3Dynamics/**/*.h"]),
    includes = ["src"],
    deps = [
        ":Bullet3Collision",
        ":Bullet3Common",
    ],
)

cc_library(
    name = "Bullet2FileLoader",
    srcs = glob(["src/Bullet3Serialize/Bullet2FileLoader/**/*.cpp"]),
    hdrs = glob(["src/Bullet3Serialize/Bullet2FileLoader/**/*.h"]),
    includes = ["src"],
    deps = [":Bullet3Common"],
)

# Convenience umbrella for the classic rigid-body physics libraries.
cc_library(
    name = "bullet",
    deps = [
        ":BulletCollision",
        ":BulletDynamics",
        ":BulletSoftBody",
        ":LinearMath",
    ],
)
