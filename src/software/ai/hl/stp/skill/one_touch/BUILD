package(default_visibility = ["//visibility:public"])

# We force linking for all Skills so that the static variables required for the
# "factory" design pattern to work are linked in
# https://www.bfilipek.com/2018/02/static-vars-static-lib.html

cc_library(
    name = "one_touch_skill",
    srcs = [
        "one_touch_skill.cpp",
        "one_touch_skill_fsm.cpp",
    ],
    hdrs = [
        "one_touch_skill.h",
        "one_touch_skill_fsm.h",
    ],
    deps = [
        "//software/ai/hl/stp/primitive:move_primitive",
        "//software/ai/hl/stp/skill:base_skill",
        "//software/ai/hl/stp/skill/dribble:dribble_skill",
        "//software/geom/algorithms",
        "//software/geom:ray",
        "//software/math:math_functions",
        "//software/util/generic_factory",
    ],
    alwayslink = True,
)
