# Description:
#   This library provides us with a 2D physics engine
#   https://github.com/erincatto/Box2D
#   https://box2d.org/

cc_library(
    name = "box2d",
    srcs = glob(["Box2D/**/*.cpp"]),
    hdrs = glob(["Box2D/**/*.h"]),
    defines = [
        # Decrease the linear slop to 0.001 (1mm) for more accuracy
        "b2_linearSlop=0.001f",
        # Decrease the angular slop for more accuracy. This value is in radians
        # and is equal to 0.1 degrees
        "b2_angularSlop=0.00174532f",
        # Decrease the velocity threshold so that have normal collisions at or above
        # this speed (1mm/s). Collisions below this speed will "stick", but we can
        # tolerate those since we don't care about anything that slow
        "b2_velocityThreshold=0.001f",
    ],
    includes = [
        ".",
    ],
    visibility = ["//visibility:public"],
)
