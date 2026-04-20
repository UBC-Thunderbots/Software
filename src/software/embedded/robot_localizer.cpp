#include "robot_localizer.h"

RobotLocalizer::RobotLocalizer(const double process_noise_variance,
                               const double vision_noise_variance,
                               const double motor_sensor_noise_variance,
                               const double target_angular_acceleration_variance)
    : process_noise_variance_(process_noise_variance)
{
    // Initial covariance is tuned for orientation, angular velocity, and acceleration
    filter_.state_covariance = Eigen::Vector<double, STATE_SIZE>(30, 4, 5).asDiagonal();

    filter_.control_model.setZero();
    filter_.control_model(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;

    filter_.measurement_covariance =
        Eigen::Vector<double, MEASUREMENT_SIZE>(
            vision_noise_variance, motor_sensor_noise_variance, ImuService::IMU_VARIANCE,
            target_angular_acceleration_variance)
            .asDiagonal();

    last_step_time_ = std::chrono::system_clock::now();
}

void RobotLocalizer::step(const AngularVelocity& target_acceleration)
{
    FilterStep step{
        .prediction       = FilterStep::Predict{},
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::system_clock::now(),
    };

    const std::chrono::duration<double> delta_time = step.time - last_step_time_;
    const double delta_time_seconds                = delta_time.count();
    last_step_time_                                = step.time;

    // clang-format off
    step.prediction->process_model <<
        1, delta_time_seconds, delta_time_seconds * delta_time_seconds * 0.5,
        0, 1, delta_time_seconds,
        0, 0, 1;
    // clang-format on

    const double delta_time_squared = delta_time_seconds * delta_time_seconds;
    const double delta_time_cubed   = delta_time_squared * delta_time_seconds;
    const double delta_time_fourth  = delta_time_cubed * delta_time_seconds;

    // clang-format off
    step.prediction->process_covariance <<
        delta_time_fourth / 4, delta_time_cubed / 3, delta_time_squared / 2,
        delta_time_cubed / 3, delta_time_squared, delta_time_seconds,
        delta_time_squared / 2, delta_time_seconds, 1;
    // clang-format on

    step.prediction->process_covariance *= process_noise_variance_;

    step.prediction->control_model = filter_.control_model;

    // Control input represents change in angular velocity over this step.
    step.prediction->control_input
        << target_acceleration.toRadians() * delta_time_seconds;

    filter_.process_model      = step.prediction->process_model;
    filter_.process_covariance = step.prediction->process_covariance;

    history.push_front(step);
    filter_.predict(step.prediction->control_input);
}

void RobotLocalizer::rollbackVision(const Angle& orientation, const double age_seconds)
{
    if (history.empty())
    {
        updateVision(orientation);
        return;
    }

    const auto current_time = std::chrono::system_clock::now();
    const auto sample_age   = std::chrono::duration<double>(age_seconds);

    const auto rollback_point = std::find_if(
        history.begin(), history.end(),
        [&](const FilterStep& step) { return step.time - current_time >= sample_age; });

    if (rollback_point == history.begin())
    {
        // All history predates the sample, no need to rollback
        updateVision(orientation);
        return;
    }

    auto replay_iter = std::make_reverse_iterator(rollback_point);

    // Truncate history after the rollback point
    history.erase(rollback_point, history.end());

    filter_.state_estimate   = replay_iter->state_estimate;
    filter_.state_covariance = replay_iter->state_covariance;

    updateVision(orientation);

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

void RobotLocalizer::updateVision(const Angle& orientation)
{
    const double orientation_estimate =
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ORIENTATION));

    Eigen::Vector<double, MEASUREMENT_SIZE> measurement;
    measurement.setZero();
    measurement(static_cast<Eigen::Index>(MeasurementIndex::VISION_ORIENTATION)) =
        orientation_estimate +
        (orientation - Angle::fromRadians(orientation_estimate)).clamp().toRadians();

    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::VISION_ORIENTATION),
        static_cast<Eigen::Index>(StateIndex::ORIENTATION)) = 1;

    filter_.update(measurement);
}

void RobotLocalizer::updateMotorSensors(const AngularVelocity& angular_velocity)
{
    FilterStep step{
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::system_clock::now(),
    };

    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::MOTOR_ANGULAR_VELOCITY),
        static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;

    step.update = FilterStep::Update{
        .measurement_model = filter_.measurement_model,
        .measurement       = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(),
    };

    step.update->measurement(static_cast<Eigen::Index>(
        MeasurementIndex::MOTOR_ANGULAR_VELOCITY)) = angular_velocity.toRadians();

    history.push_front(step);
    filter_.update(step.update->measurement);
}

void RobotLocalizer::updateImu(const AngularVelocity& angular_velocity)
{
    FilterStep step{
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::system_clock::now(),
    };

    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::IMU_ANGULAR_VELOCITY),
        static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)) = 1;

    step.update = FilterStep::Update{
        .measurement_model = filter_.measurement_model,
        .measurement       = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(),
    };

    step.update->measurement(static_cast<Eigen::Index>(
        MeasurementIndex::IMU_ANGULAR_VELOCITY)) = angular_velocity.toRadians();

    history.push_front(step);
    filter_.update(step.update->measurement);
}

void RobotLocalizer::updateTargetAcceleration(const AngularVelocity& angular_acceleration)
{
    FilterStep step{
        .state_estimate   = filter_.state_estimate,
        .state_covariance = filter_.state_covariance,
        .time             = std::chrono::system_clock::now(),
    };

    filter_.measurement_model.setZero();
    filter_.measurement_model(
        static_cast<Eigen::Index>(MeasurementIndex::TARGET_ANGULAR_ACCELERATION),
        static_cast<Eigen::Index>(StateIndex::ANGULAR_ACCELERATION)) = 1;

    step.update = FilterStep::Update{
        .measurement_model = filter_.measurement_model,
        .measurement       = Eigen::Vector<double, MEASUREMENT_SIZE>::Zero(),
    };

    step.update->measurement(
        static_cast<Eigen::Index>(MeasurementIndex::TARGET_ANGULAR_ACCELERATION)) =
        angular_acceleration.toRadians();

    history.push_front(step);
    filter_.update(step.update->measurement);
}

Angle RobotLocalizer::getOrientation() const
{
    return Angle::fromRadians(
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ORIENTATION)));
}

AngularVelocity RobotLocalizer::getAngularVelocity() const
{
    return AngularVelocity::fromRadians(
        filter_.state_estimate(static_cast<Eigen::Index>(StateIndex::ANGULAR_VELOCITY)));
}

double RobotLocalizer::getAngularAccelerationRadians() const
{
    return filter_.state_estimate(
        static_cast<Eigen::Index>(StateIndex::ANGULAR_ACCELERATION));
}
