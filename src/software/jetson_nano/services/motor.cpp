#include "software/jetson_nano/services/motor.h"

#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>

#include "proto/tbots_software_msgs.pb.h"

MotorService::MotorService(const std::string& spi_device)
{
    // TODO RAII, open all the SPI ports
}

MotorService::~MotorService()
{
    // TODO close all SPI ports
}

std::unique_ptr<TbotsProto::DriveUnitStatus> MotorService::poll(
    const TbotsProto::DirectControlPrimitive_DirectPerWheelControl& per_wheel_rpms)
{
    // TODO communicate velocities to trinamic and read back feedback
    return std::make_unique<TbotsProto::DriveUnitStatus>();
}


void MotorService::start() {}

void MotorService::stop() {}
