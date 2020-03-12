sh_binary(
    name = "flash_firmware",
    srcs = ["flash_firmware.sh"],
    data = [
        "//firmware:firmware_old_main_bin",
        "//firmware/dongle:firmware_old_radio_dongle_bin",
    ],
    restricted_to = ["//cc_toolchain:stm32f4"],
    deps = ["@bazel_tools//tools/bash/runfiles"],
)

cc_binary(
    name = "send_proto_over_radio",
    srcs = ["send_proto_over_radio.cpp"],
    restricted_to = ["//cc_toolchain:k8"],
    deps = [
        "//shared/proto:fw_cc_proto",
        "//software:constants",
        "//software/backend:radio_backend",
        "//software/backend/output/radio:radio_output",
        "@boost//:asio",
        "@com_google_protobuf//:protobuf",
    ],
)
