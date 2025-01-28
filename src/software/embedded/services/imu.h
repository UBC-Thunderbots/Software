#pragma once

#include <utility>
#include <Eigen/Dense>


#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/angular_velocity.h"

/**
 * Handles low level IMU i2c communication, and some minor offset filtering.
 */
class ImuService {
public:
    ImuService();

    /**
     * Polls the IMU to return latest reading on the first time derivative of the heading/yaw.
     * @return current measured AngularVelocity of the robot.
     */
    std::optional<AngularVelocity> pollHeadingRate();

    // Variance from datasheet
    static constexpr double IMU_VARIANCE = (4.0*14.4222/1000.0)*(4.0*14.4222/1000.0); //stdev = 4mdeg/Hz noise density * sqrt(208) Hz * 1/1000 deg/mdeg
private:
    bool initialized_ = false;
    int file_descriptor_ = 0;
    double degrees_error_;

    static constexpr double ACCELEROMETER_FULL_SCALE_G = 2.0;
    // If this is changed, the IMU config needs to be different.
    static constexpr double IMU_FULL_SCALE_DPS = 1000.0;
};
