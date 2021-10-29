#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

#include "software/jetson_nano/services/service.h"

MotorService::MotorService()
{
    // TODO RAII, open all the ports
}

std::unique_ptr<DriveUnitStatus> MotorService::poll(
    std::unique_ptr<TbotsProto::DirectControlPrimitive_DirectPerWheelControl>
        per_wheel_rpms)
{

}


MotorService::start() {}

MotorService::stop() {}
