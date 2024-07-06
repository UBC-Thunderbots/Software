#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(Platform, PI, NANO);

static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_0_GPIO = "51";
static const char* SPI_CS_DRIVER_TO_CONTROLLER_MUX_1_GPIO = "76";
static const char* MOTOR_DRIVER_RESET_GPIO                = "168";
static const char* DRIVER_CONTROL_ENABLE_GPIO             = "194";

constexpr Platform PLATFORM = Platform::NANO;
