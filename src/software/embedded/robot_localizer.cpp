#include "robot_localizer.h"
#include "shared/constants.h"

#include <iostream>

#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"

void RobotLocalizer::step(const AngularVelocity& targetAcceleration) {
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    timespec diff = current_time_;
    ScopedTimespecTimer::timespecDiff(&current_time_, &last_step_, &diff);
    clock_gettime(CLOCK_MONOTONIC, &last_step_);
    double delta_time_seconds = (double)diff.tv_sec + (double)diff.tv_nsec * SECONDS_PER_NANOSECOND;; // time since last step in seconds
    FilterStep this_step;
    Predict this_predict;
    this_step.birthday = current_time_;
    this_step.pre_mean = filter_.x;
    this_step.pre_covariance = filter_.P;
    filter_.F << 1, delta_time_seconds, delta_time_seconds * delta_time_seconds * 0.5,
            0, 1,                delta_time_seconds,
            0, 0,                1;
    double time_squared = delta_time_seconds * delta_time_seconds;
    double time_cubed = time_squared * delta_time_seconds;
    double time_fourth = time_cubed * delta_time_seconds;
    filter_.Q << time_fourth / 4, time_cubed / 3, time_squared / 2,
            time_cubed / 3,  time_squared,      delta_time_seconds,
            time_squared / 2,delta_time_seconds, 1;
    filter_.Q *= process_noise_variance_;
    Eigen::Matrix<double, 1, 1> u;
    u << targetAcceleration.toRadians() * delta_time_seconds; // change in angular velocity due to control input
    this_predict.F = filter_.F;
    this_predict.Q = filter_.Q;
    this_predict.B = filter_.B;
    this_predict.u = u;
    this_step.prediction = std::optional<Predict>(this_predict);
    history.push_front(this_step);
    filter_.predict(u);
}

void RobotLocalizer::rollbackVision(const Angle& orientation, const double& ageSeconds) {
    // Since this update is old, we roll back to when it would have come in, then recompute the filter steps since then
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    if (history.empty()) {
        updateVision(orientation);
        return;
    }
    auto it = history.begin();
    while (it != history.end()) {
        timespec this_age = current_time_;
        ScopedTimespecTimer::timespecDiff(&current_time_, &it->birthday, &this_age);
        double this_age_seconds = (double)this_age.tv_sec + (double)this_age.tv_nsec * SECONDS_PER_NANOSECOND;
        if (this_age_seconds >= ageSeconds) { // now iterator is right before the new update
            if (it == history.begin()) {
                updateVision(orientation);
                return;
            }
            --it;
            break;
        }
        ++it;

    }
     // return iterator to earliest point after new vision update
    // prune history of everything before iterator

    while(history.end() - 1 != it) {
        history.pop_back();
    }
    filter_.x = it->pre_mean;
    filter_.P = it->pre_covariance;
    updateVision(orientation);
    // roll back to the present
    do {
        if (it->prediction.has_value()) {
            Predict p = it->prediction.value();
            filter_.B = p.B;
            filter_.Q = p.Q;
            filter_.F = p.F;
            filter_.predict(p.u);
        } else {
            Update u = it->update.value();
            filter_.H = u.H;
            filter_.update(u.z);
        }
        if (it != history.begin()) {
            it--;
        }
    } while (it != history.begin());
}

void RobotLocalizer::updateVision(const Angle& orientation) {
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << filter_.x(0, 0) + (orientation - Angle::fromRadians(filter_.x(0, 0))).clamp().toRadians(),
            0, 0, 0; // this is the coterminal angle of orientation closest to x[0,0]
    filter_.H << 1, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;
    filter_.update(z);
}

void RobotLocalizer::updateEncoders(const AngularVelocity& angularVelocity) {
    FilterStep this_step;
    Update this_update;
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    this_step.birthday = current_time_;
    this_step.pre_mean = filter_.x;
    this_step.pre_covariance = filter_.P;
    filter_.H << 0, 0, 0,
                 0, 1, 0,
                 0, 0, 0,
                 0, 0, 0;
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << 0, angularVelocity.toRadians(), 0, 0;
    this_update.z = z;
    this_update.H = filter_.H;
    this_step.update = std::optional<Update>(this_update);
    history.push_front(this_step);
    filter_.update(z);
}
void RobotLocalizer::updateImu(const AngularVelocity& angularVelocity) {
    FilterStep this_step;
    Update this_update;
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    this_step.birthday = current_time_;
    this_step.pre_mean = filter_.x;
    this_step.pre_covariance = filter_.P;
    filter_.H << 0, 0, 0,
            0, 0, 0,
            0, 1, 0,
            0, 0, 0;
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << 0, 0, angularVelocity.toRadians(), 0;
    this_update.z = z;
    this_update.H = filter_.H;
    this_step.update = std::optional<Update>(this_update);
    history.push_front(this_step);
    filter_.update(z);
}

Angle RobotLocalizer::getOrientation() {
    return Angle::fromRadians(filter_.x(0, 0));
}

AngularVelocity RobotLocalizer::getAngularVelocity() {
    return AngularVelocity::fromRadians(filter_.x(1, 0));
}

double RobotLocalizer::getAngularAccelerationRadians() {
    return filter_.x(2, 0);
}

void RobotLocalizer::updateTargetAcceleration(const AngularVelocity& angularAcceleration) {
    FilterStep this_step;
    Update this_update;
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    this_step.birthday = current_time_;
    this_step.pre_mean = filter_.x;
    this_step.pre_covariance = filter_.P;
    filter_.H << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 1;
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << 0, 0, 0, angularAcceleration.toRadians();
    this_update.z = z;
    this_update.H = filter_.H;
    this_step.update = std::optional<Update>(this_update);
    history.push_front(this_step);
    filter_.update(z);
}


