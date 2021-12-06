package(default_visibility = [
    "//software/HRVO:__subpackages__",
    "//software/ai:__subpackages__",
])

cc_test(
    name = "hrvo_test",
    srcs = [
        "hrvo_test.cpp",
    ],
    deps = [
        ":hrvo",
        "//shared/test_util:tbots_gtest_main",
    ],
)

cc_library(
    name = "hrvo",
    srcs = [
        "Agent.cpp",
        "Goal.cpp",
        "KdTree.cpp",
        "Simulator.cpp",
        "Vector2.cpp",
    ],
    hdrs = [
        "Agent.h",
        "Goal.h",
        "KdTree.h",
        "Simulator.h",
        "Vector2.h",
        "Definitions.h",
        #        "HRVO.h",
    ],
    #    copts = [
    #        "-fvisibility-inlines-hidden",
    #        "-fvisibility=hidden",
    #    ],
    #    includes = ["."],
    visibility = ["//visibility:public"],
)

#filegroup(
#    name = "hdrs",
#    srcs = [
#        # "Export.h",
#        "HRVO.h",
#        "Simulator.h",
#        "Vector2.h",
#    ],
#    visibility = ["//visibility:private"],
#)

#cc_library(
#    name = "hrvo",
#    srcs = [
#        "Agent.cpp",
#        "Agent.h",
#        "Definitions.h",
#        "Goal.cpp",
#        "Goal.h",
#        "KdTree.cpp",
#        "KdTree.h",
#        "Simulator.cpp",
#        "Vector2.cpp",
#    ],
#    hdrs = [":hdrs"],
#    copts = [
#        "-fvisibility-inlines-hidden",
#        "-fvisibility=hidden",
#    ],
#    includes = ["."],
#    visibility = ["//visibility:public"],
#)

#############################################
# cc_library(
#     name = "Agent",
#     srcs = ["Agent.cpp"],
#     hdrs = ["Agent.h"],
#     deps = [
#         ":Vector2",
#         ":Goal",
#         ":Simulator",
#     ],
# )

# cc_library(
#     name = "Definition",
#     hdrs = ["Definition.h"],
# )

# cc_library(
#     name = "Goal",
#     srcs = ["Goal.cpp"],
#     hdrs = ["Goal.h"],
#     deps = [":Vector2"],
# )

# cc_library(
#     name = "KdTree",
#     srcs = ["KdTree.cpp"],
#     hdrs = ["KdTree.h"],
#     deps = [
#         ":Vector2",
#         ":Agent",
#     ],
# )

# cc_library(
#     name = "Simulator",
#     srcs = ["Simulator.cpp"],
#     hdrs = ["Simulator.h"],
#     deps = [
#         ":Vector2",
#         ":Goal",
#         ":Agent",
#         ":KdTree",
#     ],
# )

# cc_library(
#     name = "Vector2",
#     srcs = ["Vector2.cpp"],
#     hdrs = ["Vector2.h"],
#     # deps = [":Vector2"],
# )

# cc_library(
#     name = "HRVO",
#     hdrs = ["HRVO.h"],
#     deps = [
#         ":Simulator",
#         ":Vector2",
#     ],
# )
