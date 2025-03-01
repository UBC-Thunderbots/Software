#pragma once

#include <Eigen/Dense>
#include <deque>
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/sensor_fusion/filter/kalman_filter.h"
#include "proto/world.pb.h"
#include "time.h"
#include "software/embedded/services/imu.h"

/**
 * This class implements the KalmanFilter class to track robot orientation. It keeps a rolling history of the robot's state, so when vision updates come in, the filter can roll back and recompute with them.
 *
 * For information on the math and design of kalman filters, see relevant links in kalman_filter.h
 */
class RobotLocalizer {
public:
    RobotLocalizer(double process_noise_variance, double vision_noise_variance, double encoder_noise_variance, double target_angular_acceleration_variance):
    filter_(),
    process_noise_variance_(process_noise_variance),// assumes process noise is a discrete time wiener process
    history()
    {
        // Set state covariance, this is mostly a tuned value. The digaonal is the variance of the Orientation, Ang Vel, and Ang Accel
        filter_.P << 30, 0,  0,
                     0,  4, 0,
                     0,  0,  5;
        // Set control to state matrix, control space is a 1x1 matrix
        filter_.B << 0.0, 1, 0.0;
        // Set measurement variance.
        filter_.R << vision_noise_variance, 0.0,                    0.0,                      0.0,
                     0.0,                   encoder_noise_variance, 0.0,                      0.0,
                     0.0,                   0.0,                    ImuService::IMU_VARIANCE, 0.0,
                     0.0,                   0.0,                    0.0,                      target_angular_acceleration_variance;
        filter_.x = Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0);
        clock_gettime(CLOCK_MONOTONIC, &current_time_);
        clock_gettime(CLOCK_MONOTONIC, &last_step_);

    }

    void step(const AngularVelocity& targetAcceleration);

    /** Update the orientation from vision based on an old reading.
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     * @param ageSeconds Age in seconds of the vision snapshot (time since it was taken)
     */
    void rollbackVision(const Angle& orientation, const double& ageSeconds);

    /** Update the orientation from vision.
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     */
    void updateVision(const Angle& orientation);

    /** Update the angular velocity from encoders.
     *
     * @param angularVelocity angular velocity of the robot
     */
    void updateEncoders(const AngularVelocity& angularVelocity);

    /** Update the angular velocity from IMU.
     *
     * @param angularVelocity angular velocity of the robot
     */
    void updateImu(const AngularVelocity& angularVelocity);

    /** Update the target acceleration.
     *
     * @param angularAcceleration Target angular acceleration of the robot.
     */
    void updateTargetAcceleration(const AngularVelocity& angularAcceleration);

    /**
     * Gets estimated orientation of the robot.
     * @return estimated orientation of the robot.
     */
    Angle getOrientation();

    /**
     * Gets estimated angular velocity of the robot.
     * @return estimated angular velocity of the robot.
     */
    AngularVelocity getAngularVelocity();

    /**
     * Gets estimated angular acceleration of the robot.
     * @return estimated angular acceleration of the robot.
     */
    double getAngularAccelerationRadians();

private:
    /**
     * Structures for storing history of filter states.
     */
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
    // The kalman filter has two main vectors, currently we are tracking in 3 dimensions and measuring in 4.
    // State space vector:
    // x = [
    //    Orientation,
    //    Angular velocity,
    //    Angular acceleration
    // ]
    // Measurement space vector:
    // z = [
    //    Camera orientation,
    //    Wheel encoder angular velocity,
    //    IMU angular velocity,
    //    Target angular acceleration
    // ]
    KalmanFilter<3, 4, 1> filter_;

    // The variance of the process. The process is our prediction of the future.
    double process_noise_variance_;

    timespec current_time_;
    timespec last_step_;

    std::deque<FilterStep> history; // where 1st entry = newest
};
