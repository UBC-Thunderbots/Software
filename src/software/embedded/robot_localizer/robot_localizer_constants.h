#pragma once

#include <Eigen/Dense>

#include "software/util/make_enum/make_enum.hpp"

MAKE_ENUM(StateIndex, X_POSITION, Y_POSITION, ORIENTATION, X_VELOCITY, Y_VELOCITY,
          ANGULAR_VELOCITY);

MAKE_ENUM(MeasurementIndex, VISION_X_POSITION, VISION_Y_POSITION, VISION_ORIENTATION,
          MOTOR_X_VELOCITY, MOTOR_Y_VELOCITY, MOTOR_ANGULAR_VELOCITY,
          IMU_ANGULAR_VELOCITY);

MAKE_ENUM(ControlIndex, X_VELOCITY_TARGET, Y_VELOCITY_TARGET);

MAKE_ENUM(FilterStepType, PREDICT, MOTOR_DATA, IMU_DATA, VISION_DATA);


static constexpr size_t STATE_SIZE       = reflective_enum::size<StateIndex>();
static constexpr size_t MEASUREMENT_SIZE = reflective_enum::size<MeasurementIndex>();
static constexpr size_t CONTROL_SIZE     = reflective_enum::size<ControlIndex>();

// Initial Covariances
static constexpr double VISION_X_INITIAL_VARIANCE_M       = 0.00001;
static constexpr double VISION_Y_INITIAL_VARIANCE_M       = 0.00001;
static constexpr double VISION_THETA_INITIAL_VARIANCE_RAD = 0.00001;

static constexpr double MOTOR_X_INITIAL_VARIANCE_M_S       = 0.5;
static constexpr double MOTOR_Y_INITIAL_VARIANCE_M_S       = 0.5;
static constexpr double MOTOR_THETA_INITIAL_VARIANCE_RAD_S = 0.5;

static constexpr double PROCESS_MODEL_INITIAL_VARIANCE = 1;


static constexpr double PROCESS_LINEAR_VELOCITY_NOISE_VARIANCE      = 1;
static constexpr double PROCESS_ANGULAR_ACCELERATION_NOISE_VARIANCE = 1;

// Measurement models (H). Rows: MeasurementIndex, columns: StateIndex
// (X_POSITION, Y_POSITION, ORIENTATION, X_VELOCITY, Y_VELOCITY, ANGULAR_VELOCITY)
// clang-format off
static const Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> VISION_MEASUREMENT_MODEL = []
{
    Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> m;
    m << 1, 0, 0, 0, 0, 0,  // VISION_X_POSITION
         0, 1, 0, 0, 0, 0,  // VISION_Y_POSITION
         0, 0, 1, 0, 0, 0,  // VISION_ORIENTATION
         0, 0, 0, 0, 0, 0,  // MOTOR_X_VELOCITY
         0, 0, 0, 0, 0, 0,  // MOTOR_Y_VELOCITY
         0, 0, 0, 0, 0, 0,  // MOTOR_ANGULAR_VELOCITY
         0, 0, 0, 0, 0, 0;  // IMU_ANGULAR_VELOCITY
    return m;
}();

static const Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> MOTOR_MEASUREMENT_MODEL = []
{
    Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> m;
    m << 0, 0, 0, 0, 0, 0,  // VISION_X_POSITION
         0, 0, 0, 0, 0, 0,  // VISION_Y_POSITION
         0, 0, 0, 0, 0, 0,  // VISION_ORIENTATION
         0, 0, 0, 1, 0, 0,  // MOTOR_X_VELOCITY
         0, 0, 0, 0, 1, 0,  // MOTOR_Y_VELOCITY
         0, 0, 0, 0, 0, 1,  // MOTOR_ANGULAR_VELOCITY
         0, 0, 0, 0, 0, 0;  // IMU_ANGULAR_VELOCITY
    return m;
}();

static const Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> IMU_MEASUREMENT_MODEL = []
{
    Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> m;
    m << 0, 0, 0, 0, 0, 0,  // VISION_X_POSITION
         0, 0, 0, 0, 0, 0,  // VISION_Y_POSITION
         0, 0, 0, 0, 0, 0,  // VISION_ORIENTATION
         0, 0, 0, 0, 0, 0,  // MOTOR_X_VELOCITY
         0, 0, 0, 0, 0, 0,  // MOTOR_Y_VELOCITY
         0, 0, 0, 0, 0, 0,  // MOTOR_ANGULAR_VELOCITY
         0, 0, 0, 0, 0, 1;  // IMU_ANGULAR_VELOCITY
    return m;
}();
// clang-format on
