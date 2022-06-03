package(default_visibility = ["//visibility:public"])

cc_library(
	name = "encoder_spi",
	srcs = [
		"encoder_spi.cpp",
	],
	hdrs = [
		"encoder_spi.h",
	],
	deps = [
		"//software/logger:logger",
	]
)

cc_library(
	name = "encoder",
	srcs = [
		"encoder.cpp",
		"as5047spi.cpp",
	],
	hdrs = [
		"encoder.h",
		"as5047spi.h",
	],
	deps = [
		":encoder_spi"
	],
)
