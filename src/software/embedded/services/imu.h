#pragma once

#include <Eigen/Dense>
#include <utility>

#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/angular_velocity.h"

/**
 * Handles low level IMU I2C communication, and some minor offset filtering.
 */
class ImuService
{
   public:
    /**
     * Constructs and initializes a new IMU service object.
     *
     * If successfully initialized, will try to do a simple calibration of the IMU.
     */
    ImuService();

    /**
     * Polls the IMU to return latest reading on the first time derivative of the
     * heading/yaw.
     * @return current measured AngularVelocity of the robot.
     */
    std::optional<AngularVelocity> pollHeadingRate();

    // Variance from datasheet
    static constexpr double IMU_VARIANCE =
        (4.0 * 14.4222 / 1000.0) *
        (4.0 * 14.4222 /
         1000.0);  // stdev = 4mdeg/Hz noise density * sqrt(208) Hz * 1/1000 deg/mdeg
   private:
    bool initialized_    = false;
    int file_descriptor_ = 0;
    double degrees_error_;

    // The full scale of the accelerometer, that is, for any accelerometer output x, the
    // acceleration = x / SHRT_MAX * ACCELEROMETER_FULL_SCALE_G
    static constexpr double ACCELEROMETER_FULL_SCALE_G = 2.0;
    // If this is changed, the IMU config needs to be different.
    // The full scale of the IMU, that is, for IMU output a, angular velocity in DPS = a /
    // SHRT_MAX * IMU_FULL_SCALE_DPS
    static constexpr double IMU_FULL_SCALE_DPS = 1000.0;

    // Device path for the IMU
    inline static const std::string IMU_DEVICE = "/dev/i2c-1";

    // Various i2c registers.
    static const uint8_t WHOAMI_REG        = 0xf;
    static const uint8_t ACCEL_CONTROL_REG = 0x10;
    static const uint8_t GYRO_CONTROL_REG  = 0x11;
    static const uint8_t YAW_LEAST_SIG_REG = 0x26;
    static const uint8_t YAW_MOST_SIG_REG  = 0x27;
};
