#include "robot_localizer.h"

#include <chrono>

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

    last_step_time_ = std::chrono::steady_clock::now();
}

void RobotLocalizer::step(const Vector& linear_acceleration)
{
    FilterStep step{
        .prediction       = FilterStep::Predict{},
        .update           = std::nullopt,
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::steady_clock::now(),
    };

    const std::chrono::duration<double> delta_time = step.time - last_step_time_;
    const double delta_time_seconds                = delta_time.count();
    last_step_time_                                = step.time;

    // clang-format off
    step.prediction->process_model <<
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

    auto& process_covariance = step.prediction->process_covariance;
    process_covariance.setZero();

    process_covariance(static_cast<Eigen::Index>(StateIndex::X_POSITION),
                       static_cast<Eigen::Index>(StateIndex::X_POSITION)) =
        delta_time_fourth / 4 * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::X_POSITION),
                       static_cast<Eigen::Index>(StateIndex::X_VELOCITY)) =
        delta_time_cubed / 2 * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::X_VELOCITY),
                       static_cast<Eigen::Index>(StateIndex::X_POSITION)) =
        delta_time_cubed / 2 * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::X_VELOCITY),
                       static_cast<Eigen::Index>(StateIndex::X_VELOCITY)) =
        delta_time_squared * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::Y_POSITION),
                       static_cast<Eigen::Index>(StateIndex::Y_POSITION)) =
        delta_time_fourth / 4 * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::Y_POSITION),
                       static_cast<Eigen::Index>(StateIndex::Y_VELOCITY)) =
        delta_time_cubed / 2 * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY),
                       static_cast<Eigen::Index>(StateIndex::Y_POSITION)) =
        delta_time_cubed / 2 * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::Y_VELOCITY),
                       static_cast<Eigen::Index>(StateIndex::Y_VELOCITY)) =
        delta_time_squared * process_linear_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::ORIENTATION),
                       static_cast<Eigen::Index>(StateIndex::ORIENTATION)) =
        delta_time_fourth / 4 * process_angular_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::ORIENTATION),
                       static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) =
        delta_time_cubed / 2 * process_angular_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY),
                       static_cast<Eigen::Index>(StateIndex::ORIENTATION)) =
        delta_time_cubed / 2 * process_angular_acceleration_noise_variance_;

    process_covariance(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY),
                       static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) =
        delta_time_squared * process_angular_acceleration_noise_variance_;

    auto& control_model = step.prediction->control_model;
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

    step.prediction->control_input << linear_acceleration.x(), linear_acceleration.y();

    filter_.process_model      = step.prediction->process_model;
    filter_.process_covariance = step.prediction->process_covariance;
    filter_.control_model      = step.prediction->control_model;

    history.push_front(step);
    filter_.predict(step.prediction->control_input);
}

void RobotLocalizer::update(const VisionData& data)
{
    if (history.empty())
    {
        updateFilterWithVision(data.position, data.orientation);
        return;
    }

    const auto current_time = std::chrono::steady_clock::now();
    const auto sample_age   = std::chrono::duration<double>(data.age_seconds);

    const auto rollback_point = std::find_if(
        history.begin(), history.end(),
        [&](const FilterStep& step) { return step.time - current_time >= sample_age; });

    if (rollback_point == history.begin())
    {
        // All history predates the sample, no need to rollback
        updateFilterWithVision(data.position, data.orientation);
        return;
    }

    auto replay_iter = std::make_reverse_iterator(rollback_point);

    // Truncate history after the rollback point
    history.erase(rollback_point, history.end());

    filter_.state_estimate   = replay_iter->state_estimate;
    filter_.state_covariance = replay_iter->state_covariance;

    updateFilterWithVision(data.position, data.orientation);

    // Replay from the rollback point back to the current estimate
    for (; replay_iter != history.rbegin(); --replay_iter)
    {
        if (replay_iter->prediction.has_value())
        {
            const auto& prediction     = replay_iter->prediction.value();
            filter_.process_model      = prediction.process_model;
            filter_.process_covariance = prediction.process_covariance;
            filter_.control_model      = prediction.control_model;
            filter_.predict(prediction.control_input);
        }

        if (replay_iter->update.has_value())
        {
            const auto& update        = replay_iter->update.value();
            filter_.measurement_model = update.measurement_model;
            filter_.update(update.measurement);
        }
    }
}

void RobotLocalizer::updateFilterWithVision(const Point& position,
                                            const Angle& orientation)
{
    const double orientation_estimate =
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ORIENTATION));

    Eigen::Vector<double, MEASUREMENT_SIZE> measurement;
    measurement.setZero();

    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_X_POSITION)) =
        position.x();
    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_Y_POSITION)) =
        position.y();

    // Coterminal angle that is closest to current estimate
    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_ORIENTATION)) =
        orientation_estimate +
        (orientation - Angle::fromRadians(orientation_estimate)).clamp().toRadians();

    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::VISION_X_POSITION),
        static_cast<Eigen::Index>(StateIndex::X_POSITION)) = 1;
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::VISION_Y_POSITION),
        static_cast<Eigen::Index>(StateIndex::Y_POSITION)) = 1;
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::VISION_ORIENTATION),
        static_cast<Eigen::Index>(StateIndex::ORIENTATION)) = 1;

    filter_.update(measurement);
}

void RobotLocalizer::update(const MotorData& data)
{
    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::MOTOR_X_VELOCITY),
        static_cast<Eigen::Index>(StateIndex::X_VELOCITY)) = 1;
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::MOTOR_Y_VELOCITY),
        static_cast<Eigen::Index>(StateIndex::Y_VELOCITY)) = 1;
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::MOTOR_ANGULAR_VELOCITY),
        static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;

    FilterStep::Update update{
        .measurement_model = filter_.measurement_model,
        .measurement       = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(),
    };

    update.measurement(static_cast<Eigen::Index>(MeasurementIndex::MOTOR_X_VELOCITY)) =
        data.velocity.x();
    update.measurement(static_cast<Eigen::Index>(MeasurementIndex::MOTOR_Y_VELOCITY)) =
        data.velocity.y();
    update.measurement(static_cast<Eigen::Index>(
        MeasurementIndex::MOTOR_ANGULAR_VELOCITY)) = data.angular_velocity.toRadians();

    const FilterStep step{
        .prediction       = std::nullopt,
        .update           = update,
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::steady_clock::now(),
    };

    history.push_front(step);
    filter_.update(step.update->measurement);
}

void RobotLocalizer::update(const ImuData& data)
{
    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::IMU_ANGULAR_VELOCITY),
        static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;

    FilterStep::Update update{
        .measurement_model = filter_.measurement_model,
        .measurement       = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(),
    };

    update.measurement(static_cast<Eigen::Index>(
        MeasurementIndex::IMU_ANGULAR_VELOCITY)) = data.angular_velocity.toRadians();

    const FilterStep step{
        .prediction       = std::nullopt,
        .update           = update,
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::steady_clock::now(),
    };

    history.push_front(step);
    filter_.update(step.update->measurement);
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
