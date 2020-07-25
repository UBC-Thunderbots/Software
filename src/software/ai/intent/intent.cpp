#include "software/ai/intent/intent.h"

#include <algorithm>

#include "software/logger/logger.h"

// Implement concrete functions shared by all intents

Intent::Intent(unsigned int priority)
    : navigator_params_updated(false),
      navigator_params{.robot_id            = 0,
                       .motion_constraints  = std::set<MotionConstraint>(),
                       .destination         = Point(),
                       .final_speed         = 0.0,
                       .final_angle         = Angle(),
                       .ball_collision_type = BallCollisionType::ALLOW}
{
    setPriority(priority);
}

unsigned int Intent::getPriority(void) const
{
    return priority;
}

void Intent::setPriority(unsigned int new_priority)
{
    if (new_priority > 100)
    {
        LOG(WARNING) << "Intent set with out of range priority value: " << new_priority
                     << ". Clamping to range [0, 100]" << std::endl;
        new_priority = std::clamp<unsigned int>(new_priority, 0, 100);
    }
    priority = new_priority;
}

bool Intent::operator==(const Intent &other) const
{
    return this->priority == other.priority;
}

bool Intent::operator!=(const Intent &other) const
{
    return !((*this) == other);
}

void Intent::setMotionConstraints(const std::set<MotionConstraint> &motion_constraints)
{
    this->navigator_params.motion_constraints = motion_constraints;
}

std::set<MotionConstraint> Intent::getMotionConstraints(void) const
{
    return navigator_params.motion_constraints;
}

std::optional<NavigatorParams> Intent::getNavigatorParams() const
{
    if (navigator_params_updated)
    {
        return navigator_params;
    }
    return std::nullopt;
}

const std::shared_ptr<Primitive> &Intent::getPrimitive() const
{
    return primitive;
}

void Intent::updateFinalSpeedAndDestination(Point destination, double final_speed) {}

void Intent::updateNavigatorParams(unsigned int robot_id, Point destination,
                                   Angle final_angle, double final_speed,
                                   BallCollisionType ball_collision_type)
{
    navigator_params =
        NavigatorParams{.robot_id            = robot_id,
                        .motion_constraints  = navigator_params.motion_constraints,
                        .destination         = destination,
                        .final_speed         = final_speed,
                        .final_angle         = final_angle,
                        .ball_collision_type = ball_collision_type};
    navigator_params_updated = true;
}
