#include "robot_localizer.h"

#include "proto/message_translation/tbots_geometry.h"
#include "shared/constants.h"
#include "software/physics/velocity_conversion_util.h"

RobotLocalizer::RobotLocalizer(const RobotLocalizerConfig& config)
    : process_linear_acceleration_noise_variance_(config.process_noise_variance),
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

void RobotLocalizer::step(const Vector& linear_acceleration, const Duration& delta_time)
{
    const double delta_time_seconds = delta_time.toSeconds();
    current_time_seconds_ += delta_time_seconds;

    generatedPredictionMatrices(delta_time_seconds);

    Eigen::Vector<double, CONTROL_SIZE> control_input;
    control_input << linear_acceleration.x(), linear_acceleration.y();

    filter_.predict(control_input);

    history.push_front(FilterStep{
        .control_input      = control_input,
        .measurement_source = std::nullopt,
        .measurement        = std::nullopt,
        .state_estimate     = filter_.state_estimate,
        .state_covariance   = filter_.state_covariance,
        .time_seconds       = current_time_seconds_,
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
        if (it->control_input.has_value())
        {
            generatedPredictionMatrices(it->time_seconds - prev_time);
            filter_.predict(it->control_input.value());
            prev_time = it->time_seconds;
        }

        if (it->measurement.has_value())
        {
            generateMeasurementModel(it->measurement_source.value());
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
    generateMeasurementModel(MeasurementSource::VISION_DATA);

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
    generateMeasurementModel(MeasurementSource::MOTOR_DATA);

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
        .control_input      = std::nullopt,
        .measurement_source = MeasurementSource::MOTOR_DATA,
        .measurement        = measurement,
        .state_estimate     = filter_.state_estimate,
        .state_covariance   = filter_.state_covariance,
        .time_seconds       = current_time_seconds_,
    });
}

void RobotLocalizer::update(const ImuData& data)
{
    generateMeasurementModel(MeasurementSource::IMU_DATA);

    FilterStep::Update update{
        .source      = MeasurementSource::IMU_DATA,
        .measurement = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(),
    };

    update.measurement(static_cast<Eigen::Index>(
        MeasurementIndex::IMU_ANGULAR_VELOCITY)) = data.angular_velocity.toRadians();

    filter_.update(update.measurement);

    history.push_front(FilterStep{
        .prediction       = std::nullopt,
        .update           = update,
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

Vector RobotLocalizer::getVelocity() const
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
    return RobotState(getPosition(), getVelocity(), getOrientation(),
                      getAngularVelocity());
}

void RobotLocalizer::generatedPredictionMatrices(double delta_time_seconds)
{
    // clang-format off
    filter_.process_model <<
        1, 0, 0, delta_time_seconds, 0, 0,
        0, 1, 0, 0, delta_time_seconds, 0,
        0, 0, 1, 0, 0, delta_time_seconds,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    // clang-format on

    const double delta_time_squared = delta_time_seconds * delta_time_seconds;
    const double delta_time_cubed   = delta_time_squared * delta_time_seconds;
    const double delta_time_fourth  = delta_time_cubed * delta_time_seconds;

    const double linear_position_variance =
        delta_time_fourth / 4 * process_linear_acceleration_noise_variance_;
    const double linear_position_velocity_covariance =
        delta_time_cubed / 2 * process_linear_acceleration_noise_variance_;
    const double linear_velocity_variance =
        delta_time_squared * process_linear_acceleration_noise_variance_;

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

    auto& control_model = filter_.control_model;
    control_model.setZero();

    control_model(static_cast<Eigen::Index>(StateIndex::X_POSITION),
                  static_cast<Eigen::Index>(ControlIndex::X_ACCELERATION)) =
        delta_time_squared / 2;

    control_model(static_cast<Eigen::Index>(StateIndex::Y_POSITION),
                  static_cast<Eigen::Index>(ControlIndex::Y_ACCELERATION)) =
        delta_time_squared / 2;

    control_model(static_cast<Eigen::Index>(StateIndex::X_VELOCITY),
                  static_cast<Eigen::Index>(ControlIndex::X_ACCELERATION)) =
        delta_time_seconds;

    control_model(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY),
                  static_cast<Eigen::Index>(ControlIndex::Y_ACCELERATION)) =
        delta_time_seconds;
}

void RobotLocalizer::generateMeasurementModel(MeasurementSource source)
{
    filter_.measurement_model.setZero();

    switch (source)
    {
        case MeasurementSource::VISION_DATA:
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
        case MeasurementSource::MOTOR_DATA:
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
        case MeasurementSource::IMU_DATA:
            filter_.measurement_model(
                static_cast<Eigen::Index>(MeasurementIndex::IMU_ANGULAR_VELOCITY),
                static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;
            break;
    }
}
