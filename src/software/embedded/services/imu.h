#pragma once

#include <utility>
#include <Eigen/Dense>


#include "proto/tbots_software_msgs.pb.h"
#include "software/geom/angular_velocity.h"


class ImuService {
public:
    ImuService();

    std::optional<AngularVelocity> pollHeadingRate();


    static constexpr double IMU_VARIANCE = (4.0*10.19803/1000.0)*(4.0*10.19803/1000.0); //stdev = 4mdeg/Hz noise density * sqrt(104) Hz * 1/1000 deg/mdeg
private:
    bool initialized_ = false;
    int file_descriptor_ = 0;
    double degrees_error_;

    static constexpr double ACCELEROMETER_FULL_SCALE_G = 2.0;
    static constexpr double IMU_FULL_SCALE_DPS = 250.0;

    double measured_variance = 0;
    double measured_mean = 0;
    double measured_samples = 0;
};
