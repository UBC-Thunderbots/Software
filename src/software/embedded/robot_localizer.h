#pragma once

#include <Eigen/Dense>
#include <deque>

#include "proto/world.pb.h"
#include "software/embedded/services/imu.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "time.h"

/**
 * This class implements the KalmanFilter class to track robot orientation. It keeps a
 * rolling history of the robot's state, so when vision updates come in, the filter can
 * roll back and recompute with them.
 *
 * For information on the math and design of kalman filters, see relevant links in
 * kalman_filter.h
 */
class RobotLocalizer
{
   public:
    /**
     * Creates a new robot localizer. The variances given determine how much we trust each
     * source of feedback
     *
     * @param process_noise_variance
     * @param vision_noise_variance
     * @param encoder_noise_variance
     * @param target_angular_acceleration_variance
     */
    RobotLocalizer(double process_noise_variance, double vision_noise_variance,
                   double encoder_noise_variance,
                   double target_angular_acceleration_variance);

    /**
     * Innovates the state according to the time since the last time this function was
     * called.
     *
     * @param target_acceleration The target acceleration the robot is trying to attain
     * right now.
     */
    void step(const AngularVelocity& target_acceleration);

    /**
     * Update the orientation from vision based on an old reading.
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     * @param age_seconds Age in seconds of the vision snapshot (time since it was taken)
     */
    void rollbackVision(const Angle& orientation, const double& age_seconds);

    /**
     * Update the orientation from vision.
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     */
    void updateVision(const Angle& orientation);

    /**
     * Update the angular velocity from encoders.
     *
     * @param angular_velocity angular velocity of the robot
     */
    void updateEncoders(const AngularVelocity& angular_velocity);

    /**
     * Update the angular velocity from IMU.
     *
     * @param angular_velocity angular velocity of the robot
     */
    void updateImu(const AngularVelocity& angular_velocity);

    /**
     * Update the target acceleration.
     *
     * @param angular_acceleration Target angular acceleration of the robot.
     */
    void updateTargetAcceleration(const AngularVelocity& angular_acceleration);

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
    struct Predict
    {
        Eigen::Matrix<double, 3, 3> F;
        Eigen::Matrix<double, 3, 3> Q;
        Eigen::Matrix<double, 3, 1> B;
        Eigen::Matrix<double, 1, 1> u;
    };
    struct Update
    {
        Eigen::Matrix<double, 4, 3> H;
        Eigen::Matrix<double, 4, 1> z;
    };
    struct FilterStep
    {
        Eigen::Matrix<double, 3, 1> pre_mean;
        Eigen::Matrix<double, 3, 3> pre_covariance;
        std::optional<Predict> prediction;
        std::optional<Update> update;
        timespec birthday;  // time at which step was executed
    };
    // The kalman filter has two main vectors, currently we are tracking in 3 dimensions
    // and measuring in 4. State space vector: x = [
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

    std::deque<FilterStep> history;  // where 1st entry = newest
};
