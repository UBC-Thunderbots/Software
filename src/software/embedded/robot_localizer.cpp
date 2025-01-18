#include "robot_localizer.h"
#include "shared/constants.h"

#include <iostream>

#include "software/util/scoped_timespec_timer/scoped_timespec_timer.h"

void RobotLocalizer::step(const AngularVelocity& targetAcceleration) {
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    timespec diff = current_time_;
    ScopedTimespecTimer::timespecDiff(&current_time_, &last_step_, &diff);
    clock_gettime(CLOCK_MONOTONIC, &last_step_);
    double deltaTimeSeconds = (double)diff.tv_sec + (double)diff.tv_nsec * SECONDS_PER_NANOSECOND;; // time since last step in seconds
    FilterStep thisStep;
    Predict thisPredict;
    thisStep.birthday = current_time_;
    thisStep.pre_mean = filter_.x;
    thisStep.pre_covariance = filter_.P;
    filter_.F << 1, deltaTimeSeconds, deltaTimeSeconds * deltaTimeSeconds * 0.5,
            0, 1,                deltaTimeSeconds,
            0, 0,                1;
    double timeSquared = deltaTimeSeconds * deltaTimeSeconds;
    double timeCubed = timeSquared * deltaTimeSeconds;
    double timeFourth = timeCubed * deltaTimeSeconds;
    filter_.Q << timeFourth/4, timeCubed/3,      timeSquared/2,
                 timeCubed/3,  timeSquared,      deltaTimeSeconds,
                 timeSquared/2,deltaTimeSeconds, 1;
    filter_.Q *= process_noise_variance_;
    Eigen::Matrix<double, 1, 1> u;
    u << targetAcceleration.toRadians() * deltaTimeSeconds; // change in angular velocity due to control input
    thisPredict.F = filter_.F;
    thisPredict.Q = filter_.Q;
    thisPredict.B = filter_.B;
    thisPredict.u = u;
    thisStep.prediction = std::optional<Predict>(thisPredict);
    history.push_front(thisStep);
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
        timespec thisAge = current_time_;
        ScopedTimespecTimer::timespecDiff(&current_time_, &it->birthday, &thisAge);
        double thisAgeSeconds = (double)thisAge.tv_sec + (double)thisAge.tv_nsec * SECONDS_PER_NANOSECOND;
        if (thisAgeSeconds >= ageSeconds) { // now iterator is right before the new update
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
    FilterStep thisStep;
    Update thisUpdate;
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    thisStep.birthday = current_time_;
    thisStep.pre_mean = filter_.x;
    thisStep.pre_covariance = filter_.P;
    filter_.H << 0, 0, 0,
                 0, 1, 0,
                 0, 0, 0,
                 0, 0, 0;
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << 0, angularVelocity.toRadians(), 0, 0;
    thisUpdate.z = z;
    thisUpdate.H = filter_.H;
    thisStep.update = std::optional<Update>(thisUpdate);
    history.push_front(thisStep);
    filter_.update(z);
}
void RobotLocalizer::updateImu(const AngularVelocity& angularVelocity) {
    FilterStep thisStep;
    Update thisUpdate;
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    thisStep.birthday = current_time_;
    thisStep.pre_mean = filter_.x;
    thisStep.pre_covariance = filter_.P;
    filter_.H << 0, 0, 0,
            0, 0, 0,
            0, 1, 0,
            0, 0, 0;
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << 0, 0, angularVelocity.toRadians(), 0;
    thisUpdate.z = z;
    thisUpdate.H = filter_.H;
    thisStep.update = std::optional<Update>(thisUpdate);
    history.push_front(thisStep);
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
    FilterStep thisStep;
    Update thisUpdate;
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    thisStep.birthday = current_time_;
    thisStep.pre_mean = filter_.x;
    thisStep.pre_covariance = filter_.P;
    filter_.H << 0, 0, 0,
                 0, 0, 0,
                 0, 0, 0,
                 0, 0, 1;
    Eigen::Matrix<double, 4, 1> z; // heading, angular velocity
    z << 0, 0, 0, angularAcceleration.toRadians();
    thisUpdate.z = z;
    thisUpdate.H = filter_.H;
    thisStep.update = std::optional<Update>(thisUpdate);
    history.push_front(thisStep);
    filter_.update(z);
}


