#pragma once
#include <memory>
#include <string>

#include "proto/robot_status_msg.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/jetson_nano/services/service.h"

class MotorService : public Service
{
   public:
    /**
     * Service that interacts with the motor board.
     * Opens all the required ports and maintains them until destroyed.
     *
     * @param spi_device The device the motor service will interact with
     */
    explicit MotorService(const std::string& spi_device);
    ~MotorService();

    /**
     * Starts the motor service by pulling the enable pin high.
     */
    void start() override;

    /**
     * Pulls the enable pin low to disable the motor board.
     */
    void stop() override;

    /**
     * Polling the motor service will relay the provided per wheel control msg
     *
     * TODO (#TODO) Change the input here to include dribbler rpm
     *
     * @param per_wheel_rpms The target rpms for the motor board
     * @returns DriveUnitStatus The status of all the drive units
     */
    std::unique_ptr<TbotsProto::DriveUnitStatus> poll(
        const TbotsProto::DirectControlPrimitive_DirectPerWheelControl& per_wheel_rpms);

   private:
    int trinamic_spi_fd;  // The SPI file descriptor
};
