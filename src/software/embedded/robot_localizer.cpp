#include "robot_localizer.h"

#include <cmath>

#include "proto/message_translation/tbots_geometry.h"
#include "shared/constants.h"
#include "software/physics/velocity_conversion_util.h"

RobotLocalizer::RobotLocalizer(const RobotLocalizerConfig& config)
    : process_linear_velocity_noise_variance_(config.process_noise_variance),
      process_angular_acceleration_noise_variance_(config.process_noise_variance)
{
    filter_.state_covariance =
        Eigen::Vector<double, STATE_SIZE>(1, 1, 1, 1, 1, 1).asDiagonal();

    filter_.measurement_covariance =
        Eigen::Vector<double, MEASUREMENT_SIZE>(
            config.vision_noise_variance, config.vision_noise_variance,
            config.vision_noise_variance, config.motor_sensor_noise_variance,
            config.motor_sensor_noise_variance, config.motor_sensor_noise_variance,
            ImuService::IMU_VARIANCE)
            .asDiagonal();
}

void RobotLocalizer::predict(const Vector& target_velocity, const Duration& delta_time)
{
    const double delta_time_seconds = delta_time.toSeconds();
    current_time_seconds_ += delta_time_seconds;

    generatedPredictionMatrices(delta_time_seconds);

    Eigen::Vector<double, CONTROL_SIZE> control_input;
    control_input << target_velocity.x(), target_velocity.y();

    filter_.predict(control_input);

    history.push_front(FilterStep{
        .type             = FilterStepType::PREDICT,
        .control_input    = control_input,
        .measurement      = std::nullopt,
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time_seconds     = current_time_seconds_,
    });
}

void RobotLocalizer::update(const VisionData& data)
{
    if (history.empty())
    {
        updateFilterWithVision(data.position, data.orientation);
        return;
    }

    auto rollback_point = std::find_if(
        history.begin(), history.end(),
        [&](const FilterStep& step)
        { return (current_time_seconds_ - step.time_seconds) >= data.age_seconds; });

	// If rollback point is at the start, vision is newer than all history steps
	// So we empty history and apply vision
    if (rollback_point == history.begin())
    {
        updateFilterWithVision(data.position, data.orientation);
        history.clear();  // Safe to clear, since all history is older than current state
        return;
    }

	// If rollback point is at the end, vision is older than all history steps
	// So rollback ever step 
    if (rollback_point == history.end())
    {
        rollback_point = std::prev(history.end());
    }

    // 1. Roll the filter back to the state right after the rollback point's own
    //    operation ran (state is captured post-operation), since the rollback point
    //    happened at or before the vision sample's timestamp.
    filter_.state_estimate   = rollback_point->state_estimate;
    filter_.state_covariance = rollback_point->state_covariance;

    // 2. Drop the rollback point and everything older than it. Its operation is
    //    already reflected in the state we just restored, so it doesn't need to be
    //    replayed again.
    history.erase(rollback_point, history.end());

    // 3. Apply the delayed vision measurement at the rolled-back time.
    updateFilterWithVision(data.position, data.orientation);

    // 4. Replay the remaining (newer) history, from oldest to newest, recomputing each
    //    predict step's elapsed time from the vision sample's true timestamp so the
    //    inserted correction doesn't skew the replayed intervals.
    double prev_time = current_time_seconds_ - data.age_seconds;
    for (auto it = history.rbegin(); it != history.rend(); ++it)
    {
        if (it->type == FilterStepType::PREDICT)
        {
            generatedPredictionMatrices(it->time_seconds - prev_time);
            filter_.predict(it->control_input.value());
            prev_time = it->time_seconds;
        }
        else
        {
            generateMeasurementModel(it->type);
            filter_.update(it->measurement.value());
        }

        // Update the history with the recomputed state so future rollbacks are correct
        it->state_estimate   = filter_.state_estimate;
        it->state_covariance = filter_.state_covariance;
    }
}

void RobotLocalizer::updateFilterWithVision(const Point& position,
                                            const Angle& orientation)
{
    generateMeasurementModel(FilterStepType::VISION_DATA);

    const double orientation_estimate =
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ORIENTATION));

    Eigen::Vector<double, MEASUREMENT_SIZE> measurement;
    measurement.setZero();

    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_X_POSITION)) =
        position.x();
    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_Y_POSITION)) =
        position.y();

	// Integrating omega for position makes angule goes out of bounds so we wrap it around
    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_ORIENTATION)) =
        orientation_estimate +
        (orientation - Angle::fromRadians(orientation_estimate)).clamp().toRadians();


    filter_.update(measurement);
}

void RobotLocalizer::update(const MotorData& data)
{
    generateMeasurementModel(FilterStepType::MOTOR_DATA);

    Eigen::Vector<double, MEASUREMENT_SIZE> measurement =
        Eigen::Vector<double, MEASUREMENT_SIZE>::Zero();

    measurement(static_cast<Eigen::Index>(MeasurementIndex::MOTOR_X_VELOCITY)) =
        data.velocity.x();
    measurement(static_cast<Eigen::Index>(MeasurementIndex::MOTOR_Y_VELOCITY)) =
        data.velocity.y();
    measurement(static_cast<Eigen::Index>(MeasurementIndex::MOTOR_ANGULAR_VELOCITY)) =
        data.angular_velocity.toRadians();

    filter_.update(measurement);

    history.push_front(FilterStep{
        .type             = FilterStepType::MOTOR_DATA,
        .control_input    = std::nullopt,
        .measurement      = measurement,
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time_seconds     = current_time_seconds_,
    });
}

void RobotLocalizer::update(const ImuData& data)
{
    generateMeasurementModel(FilterStepType::IMU_DATA);

    Eigen::Vector<double, MEASUREMENT_SIZE> measurement =
        Eigen::Vector<double, MEASUREMENT_SIZE>::Zero();

    measurement(static_cast<Eigen::Index>(MeasurementIndex::IMU_ANGULAR_VELOCITY)) =
        data.angular_velocity.toRadians();

    filter_.update(measurement);

    history.push_front(FilterStep{
        .type             = FilterStepType::IMU_DATA,
        .control_input    = std::nullopt,
        .measurement      = measurement,
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time_seconds     = current_time_seconds_,
    });
}

Point RobotLocalizer::getPosition() const
{
    return Point(
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::X_POSITION)),
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::Y_POSITION)));
}

Vector RobotLocalizer::getGlobalVelocity() const
{
    return localToGlobalVelocity(getLocalVelocity(), getOrientation());
}

Vector RobotLocalizer::getLocalVelocity() const
{
    return Vector(
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::X_VELOCITY)),
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY)));
}

Angle RobotLocalizer::getOrientation() const
{
    return Angle::fromRadians(
               filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ORIENTATION)))
        .clamp();
}

AngularVelocity RobotLocalizer::getAngularVelocity() const
{
    return AngularVelocity::fromRadians(
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)));
}

RobotState RobotLocalizer::getRobotState() const
{
    return RobotState(getPosition(), getGlobalVelocity(), getOrientation(),
                      getAngularVelocity());
}

// TODO: Investigate proces models/variances/etc
void RobotLocalizer::generatedPredictionMatrices(double delta_time_seconds)
{
    // Velocity is estimated in the robot's local frame (see StateIndex), but position
    // is in world space, so propagating position requires rotating local velocity by
    // the current orientation estimate -- a nonlinear operation, hence the process
    // model function/Jacobian pair instead of a constant matrix.
    //
    // Velocity itself isn't propagated from its own estimate: it's replaced outright
    // by the (rotated) control input every step (see control_model below), so f(x)
    // leaves it at zero and its row of the Jacobian is zero too.
    filter_.process_model_function =
        [delta_time_seconds](Eigen::Vector<double, STATE_SIZE> state)
    {
        const double theta =
            state(static_cast<Eigen::Index>(StateIndex::ORIENTATION));
        const double local_vx =
            state(static_cast<Eigen::Index>(StateIndex::X_VELOCITY));
        const double local_vy =
            state(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY));

        Eigen::Vector<double, STATE_SIZE> next_state =
            Eigen::Vector<double, STATE_SIZE>::Zero();

        next_state(static_cast<Eigen::Index>(StateIndex::X_POSITION)) =
            state(static_cast<Eigen::Index>(StateIndex::X_POSITION)) +
            delta_time_seconds *
                (local_vx * std::cos(theta) - local_vy * std::sin(theta));
        next_state(static_cast<Eigen::Index>(StateIndex::Y_POSITION)) =
            state(static_cast<Eigen::Index>(StateIndex::Y_POSITION)) +
            delta_time_seconds *
                (local_vx * std::sin(theta) + local_vy * std::cos(theta));
        next_state(static_cast<Eigen::Index>(StateIndex::ORIENTATION)) =
            theta + delta_time_seconds *
                        state(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY));
        next_state(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) =
            state(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY));

        return next_state;
    };

    filter_.process_model_jacobian_function =
        [delta_time_seconds](Eigen::Vector<double, STATE_SIZE> state)
    {
        const auto x_position_index = static_cast<Eigen::Index>(StateIndex::X_POSITION);
        const auto y_position_index = static_cast<Eigen::Index>(StateIndex::Y_POSITION);
        const auto orientation_index =
            static_cast<Eigen::Index>(StateIndex::ORIENTATION);
        const auto x_velocity_index = static_cast<Eigen::Index>(StateIndex::X_VELOCITY);
        const auto y_velocity_index = static_cast<Eigen::Index>(StateIndex::Y_VELOCITY);
        const auto angular_velocity_index =
            static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY);

        const double theta       = state(orientation_index);
        const double local_vx    = state(x_velocity_index);
        const double local_vy    = state(y_velocity_index);
        const double cos_theta   = std::cos(theta);
        const double sin_theta   = std::sin(theta);

        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> jacobian =
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>::Identity();

        jacobian(x_position_index, orientation_index) =
            delta_time_seconds * (-local_vx * sin_theta - local_vy * cos_theta);
        jacobian(x_position_index, x_velocity_index) = delta_time_seconds * cos_theta;
        jacobian(x_position_index, y_velocity_index) = -delta_time_seconds * sin_theta;

        jacobian(y_position_index, orientation_index) =
            delta_time_seconds * (local_vx * cos_theta - local_vy * sin_theta);
        jacobian(y_position_index, x_velocity_index) = delta_time_seconds * sin_theta;
        jacobian(y_position_index, y_velocity_index) = delta_time_seconds * cos_theta;

        jacobian(orientation_index, angular_velocity_index) = delta_time_seconds;

        // f leaves velocity at zero regardless of the input state (see
        // process_model_function above), so its row of the Jacobian is zero, not the
        // identity default.
        jacobian(x_velocity_index, x_velocity_index) = 0;
        jacobian(y_velocity_index, y_velocity_index) = 0;

        return jacobian;
    };

    const double delta_time_squared = delta_time_seconds * delta_time_seconds;
    const double delta_time_cubed   = delta_time_squared * delta_time_seconds;
    const double delta_time_fourth  = delta_time_cubed * delta_time_seconds;

    // Linear terms model velocity itself as the noisy quantity (how much actual
    // velocity deviates from the commanded target velocity), integrated once into
    // position, rather than a noisy acceleration integrated twice.
    const double linear_position_variance =
        delta_time_cubed * process_linear_velocity_noise_variance_;
    const double linear_position_velocity_covariance =
        delta_time_squared * process_linear_velocity_noise_variance_;
    const double linear_velocity_variance =
        delta_time_seconds * process_linear_velocity_noise_variance_;

    // Angular terms are unchanged: angular velocity has no control input, so it's
    // still modeled as a noisy acceleration integrated twice.
    const double angular_position_variance =
        delta_time_fourth / 4 * process_angular_acceleration_noise_variance_;
    const double angular_position_velocity_covariance =
        delta_time_cubed / 2 * process_angular_acceleration_noise_variance_;
    const double angular_velocity_variance =
        delta_time_squared * process_angular_acceleration_noise_variance_;

    // State order: X_POSITION, Y_POSITION, ORIENTATION, X_VELOCITY, Y_VELOCITY,
    // ANGULAR_VELOCITY
    // clang-format off
    filter_.process_covariance <<
        linear_position_variance, 0, 0, linear_position_velocity_covariance, 0, 0,
        0, linear_position_variance, 0, 0, linear_position_velocity_covariance, 0,
        0, 0, angular_position_variance, 0, 0, angular_position_velocity_covariance,
        linear_position_velocity_covariance, 0, 0, linear_velocity_variance, 0, 0,
        0, linear_position_velocity_covariance, 0, 0, linear_velocity_variance, 0,
        0, 0, angular_position_velocity_covariance, 0, 0, angular_velocity_variance;
    // clang-format on

    // Control input is the commanded (target) linear velocity in world space: it
    // replaces the local velocity state outright, rotated into the robot's local
    // frame by the current orientation estimate (see process_model_function above,
    // which then rotates that local velocity back into world space to propagate
    // position). Position is no longer driven directly from control input here --
    // that happens through the process model function instead.
    const double theta =
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ORIENTATION));
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    auto& control_model = filter_.control_model;
    control_model.setZero();

    control_model(static_cast<Eigen::Index>(StateIndex::X_VELOCITY),
                  static_cast<Eigen::Index>(ControlIndex::X_VELOCITY_TARGET)) =
        cos_theta;
    control_model(static_cast<Eigen::Index>(StateIndex::X_VELOCITY),
                  static_cast<Eigen::Index>(ControlIndex::Y_VELOCITY_TARGET)) =
        sin_theta;

    control_model(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY),
                  static_cast<Eigen::Index>(ControlIndex::X_VELOCITY_TARGET)) =
        -sin_theta;
    control_model(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY),
                  static_cast<Eigen::Index>(ControlIndex::Y_VELOCITY_TARGET)) =
        cos_theta;
}

void RobotLocalizer::generateMeasurementModel(FilterStepType source)
{
    filter_.measurement_model.setZero();

    switch (source)
    {
        case FilterStepType::VISION_DATA:
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::VISION_X_POSITION),
                static_cast<Eigen::Index>(StateIndex::X_POSITION)) = 1;
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::VISION_Y_POSITION),
                static_cast<Eigen::Index>(StateIndex::Y_POSITION)) = 1;
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::VISION_ORIENTATION),
                static_cast<Eigen::Index>(StateIndex::ORIENTATION)) = 1;
            break;
        case FilterStepType::MOTOR_DATA:
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::MOTOR_X_VELOCITY),
                static_cast<Eigen::Index>(StateIndex::X_VELOCITY)) = 1;
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::MOTOR_Y_VELOCITY),
                static_cast<Eigen::Index>(StateIndex::Y_VELOCITY)) = 1;
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::MOTOR_ANGULAR_VELOCITY),
                static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;
            break;
        case FilterStepType::IMU_DATA:
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::IMU_ANGULAR_VELOCITY),
                static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;
            break;
        case FilterStepType::PREDICT:
            // Never called with PREDICT; predict steps use generatedPredictionMatrices.
            break;
    }
}
