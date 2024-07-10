#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(Platform, RASP_PI, JETSON_NANO);

static const int SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = 16;
static const int SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = 19;
static const int MOTOR_DRIVER_RESET_GPIO                = 12;
static const int DRIVER_CONTROL_ENABLE_GPIO             = 22;

const Platform PLATFORM = Platform::RASP_PI;