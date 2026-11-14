#pragma once

#include <cmath>

#include "software/sensor_fusion/filter/robot_filter.h"

// TODO: Test these values when we get robots?

// The robot starts out unknown, so the initial estimate is given a covariance wide
// enough to cover anywhere on the field it might be and any speed it might legally be
// moving at. This makes the filter trust the first detections it sees almost
// entirely, letting it converge onto the robot within a few frames.
constexpr double RF_INITIAL_POSITION_UNCERTAINTY_M       = 1.0;
constexpr double RF_INITIAL_VELOCITY_UNCERTAINTY_M_PER_S = 6.5;
constexpr double RF_INITIAL_ORIENTATION_UNCERTAINTY_RAD  = M_PI * M_PI / 3;

// The fastest we will believe the robot could be travelling when deciding whether a
// detection could plausibly belong to it.
constexpr double RF_MAX_ROBOT_SPEED_M_PER_S = 6.0;

// Slack on the max robot speed gate, so that vision noise on a robot that has been
// sitting still cannot by itself push a detection out of reach of the estimate
constexpr double RF_MAX_ROBOT_SPEED_GATE_TOLERANCE_M = 0.05;

// The standard deviation of the acceleration that the constant velocity motion model
// does not account for, as robots constantly accelerate
constexpr double RF_ACCELERATION_NOISE_M_PER_S_SQUARED       = 4.0;
constexpr double RF_ANG_ACCELERATION_NOISE_RAD_PER_S_SQUARED = 10.0;

// Maximum Mahalanobi's Distance before rejecting as outlier
constexpr double RF_MAHALANOBIS_GATE_THRESHOLD = 5;

// How many detections in a row may be rejected as outliers before we conclude the
// estimate itself is wrong and reset onto the newest detection
constexpr int RF_CONSECUTIVE_OUTLIERS_THRESHOLD = 3;

constexpr double RF_INITIAL_ANG_VELOCITY_UNCERTAINTY_RAD_PER_S = 4 * M_PI * M_PI;

// How noisy we expect SSL Vision's robot position detections to be. Measure this by
// logging a stationary robot and taking the standard deviation of the detections.
constexpr double RF_POS_VISION_NOISE_M   = 0.01;
constexpr double RF_ANG_VISION_NOISE_RAD = .1;


const Eigen::Vector<double, RobotFilter::STATE_SIZE> RF_INITIAL_STATE =
    Eigen::Vector<double, RobotFilter::STATE_SIZE>::Zero();
const Eigen::Matrix<double, RobotFilter::STATE_SIZE, RobotFilter::STATE_SIZE>
    RF_INITIAL_COVARIANCE =
        Eigen::Vector<double, RobotFilter::STATE_SIZE>(
            RF_INITIAL_POSITION_UNCERTAINTY_M * RF_INITIAL_POSITION_UNCERTAINTY_M,
            RF_INITIAL_POSITION_UNCERTAINTY_M* RF_INITIAL_POSITION_UNCERTAINTY_M,
            RF_INITIAL_VELOCITY_UNCERTAINTY_M_PER_S*
                RF_INITIAL_VELOCITY_UNCERTAINTY_M_PER_S,
            RF_INITIAL_VELOCITY_UNCERTAINTY_M_PER_S*
                RF_INITIAL_VELOCITY_UNCERTAINTY_M_PER_S,
            RF_INITIAL_ORIENTATION_UNCERTAINTY_RAD*
                RF_INITIAL_ORIENTATION_UNCERTAINTY_RAD,
            RF_INITIAL_ANG_VELOCITY_UNCERTAINTY_RAD_PER_S*
                RF_INITIAL_ANG_VELOCITY_UNCERTAINTY_RAD_PER_S)
            .asDiagonal();

const Eigen::Matrix<double, RobotFilter::MEASUREMENT_SIZE, RobotFilter::MEASUREMENT_SIZE>
    RF_MEASUREMENT_COVARIANCE =
        Eigen::Vector<double, RobotFilter::MEASUREMENT_SIZE>(
            RF_POS_VISION_NOISE_M * RF_POS_VISION_NOISE_M,
            RF_POS_VISION_NOISE_M* RF_POS_VISION_NOISE_M,
            RF_ANG_VISION_NOISE_RAD* RF_ANG_VISION_NOISE_RAD)
            .asDiagonal();

// Vision measures robot's position, orientation but not its velocity nor angular velocity
// clang-format off
const Eigen::Matrix<double, RobotFilter::MEASUREMENT_SIZE, RobotFilter::STATE_SIZE>
    RF_MEASUREMENT_MODEL = (Eigen::Matrix<double, RobotFilter::MEASUREMENT_SIZE,
                                          RobotFilter::STATE_SIZE>()
                                 << 1, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0,
                                    0, 0, 0, 0, 1, 0)
                               .finished();
// clang-format on
