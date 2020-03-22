package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wifi_output",
    srcs = ["wifi_output.cpp"],
    hdrs = ["wifi_output.h"],
    deps = [
        "//software/ai/primitive",
        "//software/backend:robot_status",
        "//software/backend/output/wifi/communication:protobuf_primitive_visitor",
        "//software/backend/output/wifi/communication:robot_communicator",
        "//software/world:ball",
        "//software/world:team",
    ],
)
