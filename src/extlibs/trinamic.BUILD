#   This library provides us the interface to trinamic chips

cc_library(
    name = "trinamic",
    srcs = [
        "tmc/ic/TMC4671/TMC4671.c",
        "tmc/ic/TMC6100/TMC6100.c",
    ],
    hdrs = [
        "tmc/ic/TMC4671/TMC4671.h",
        "tmc/ic/TMC4671/TMC4671_Constants.h",
        "tmc/ic/TMC4671/TMC4671_Fields.h",
        "tmc/ic/TMC4671/TMC4671_Register.h",
        "tmc/ic/TMC4671/TMC4671_Variants.h",
        "tmc/ic/TMC6100/TMC6100.h",
        "tmc/ic/TMC6100/TMC6100_Constants.h",
        "tmc/ic/TMC6100/TMC6100_Fields.h",
        "tmc/ic/TMC6100/TMC6100_Register.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":tmc-helpers-lib",
        ":tmc-ramp-lib",
    ],
)

cc_library(
    name = "tmc-helpers-lib",
    srcs = [
        "tmc/helpers/CRC.c",
        "tmc/helpers/Functions.c",
    ],
    hdrs = [
        "tmc/helpers/API_Header.h",
        "tmc/helpers/Bits.h",
        "tmc/helpers/CRC.h",
        "tmc/helpers/Config.h",
        "tmc/helpers/Constants.h",
        "tmc/helpers/Functions.h",
        "tmc/helpers/Macros.h",
        "tmc/helpers/RegisterAccess.h",
        "tmc/helpers/Types.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tmc-ramp-lib",
    srcs = [
        "tmc/ramp/LinearRamp.c",
        "tmc/ramp/LinearRamp1.c",
        "tmc/ramp/Ramp.c",
    ],
    hdrs = [
        "tmc/ramp/LinearRamp.h",
        "tmc/ramp/LinearRamp1.h",
        "tmc/ramp/Ramp.h",
    ],
    visibility = ["//visibility:public"],
    deps =
        [
            ":tmc-helpers-lib",
        ],
)
