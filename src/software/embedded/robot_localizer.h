#pragma once

#include <Eigen/Dense>
#include <deque>
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/sensor_fusion/filter/kalman_filter.h"
#include "proto/world.pb.h"
#include "time.h"
#include "software/embedded/services/imu.h"


class RobotLocalizer {
public:
    RobotLocalizer(double process_noise_variance, double vision_noise_variance, double encoder_noise_variance, double target_angular_acceleration_variance):
    filter_(),
    process_noise_variance_(process_noise_variance),// assumes process noise is a discrete time wiener process
    history()
    {
        filter_.P << 30, 0,  0,
                     0,  4, 0,
                     0,  0,  5;
        filter_.B << 0.0, 1, 0.0;
        filter_.R << vision_noise_variance, 0.0,                    0.0,0.0,
                     0.0,                   encoder_noise_variance, 0.0,0.0,
                     0.0,                   0.0,                    ImuService::IMU_VARIANCE,0.0,
                     0.0,                   0.0,                    0.0,target_angular_acceleration_variance;
        filter_.x = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        clock_gettime(CLOCK_MONOTONIC, &current_time_);
        clock_gettime(CLOCK_MONOTONIC, &last_step_);

    }

    void Step(AngularVelocity targetAcceleration);

    /**
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     * @param ageSeconds Age in seconds of the vision snapshot (time since it was taken)
     */
    void RollbackVision(Angle orientation, double ageSeconds);

    /**
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     */
    void UpdateVision(Angle orientation);

    /**
     *
     * @param angularVelocity angular velocity of the robot
     */
    void UpdateEncoders(AngularVelocity angularVelocity);

    /**
     *
     * @param angularVelocity angular velocity of the robot
     */
    void UpdateIMU(AngularVelocity angularVelocity);


    void UpdateTargetAcceleration(AngularVelocity angularAcceleration);


    Angle getOrientation();
    AngularVelocity getAngularVelocity();
    double getAngularAccelerationRadians();

private:
    struct Predict {
        Eigen::Matrix<double, 3, 3> F;
        Eigen::Matrix<double, 3, 3> Q;
        Eigen::Matrix<double, 3, 1> B;
        Eigen::Matrix<double, 1, 1> u;
    };
    struct Update {
        Eigen::Matrix<double, 4, 3> H;
        Eigen::Matrix<double, 4, 1> z;
    };
    struct FilterStep {
        Eigen::Matrix<double, 3, 1> pre_mean;
        Eigen::Matrix<double, 3, 3> pre_covariance;
        std::optional<Predict> prediction;
        std::optional<Update> update;
        timespec birthday; // time at which step was executed
    };
    // state is second order of orientation
    // measurement is camera orientation (where angular velocity is hidden), wheel encoder inverse kinematics angular velocity, imu angular velocity, and intended angular acceleration
    // control input is simply a target angular velocity
    KalmanFilter<3, 4, 1> filter_;

    double process_noise_variance_;

    timespec current_time_;
    timespec last_step_;

    std::deque<FilterStep> history; // where 1st entry = newest
};
