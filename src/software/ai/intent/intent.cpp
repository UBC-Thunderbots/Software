#include "software/ai/intent/intent.h"

#include <google/protobuf/util/message_differencer.h>

#include <algorithm>

#include "software/logger/logger.h"

Intent::Intent(unsigned int robot_id, PrimitiveMsg primitive_msg, unsigned int priority)
    : robot_id(robot_id),
      motion_constraints(),
      navigator_params(std::nullopt),
      primitive_msg(primitive_msg)
{
    setPriority(priority);
}

unsigned int Intent::getPriority(void) const
{
    return priority;
}

unsigned int Intent::getRobotId() const
{
    return robot_id;
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
    return this->priority == other.priority &&
           this->motion_constraints == other.motion_constraints &&
           this->navigator_params == other.navigator_params &&
           this->robot_id == other.robot_id &&
           google::protobuf::util::MessageDifferencer::Equals(this->primitive_msg,
                                                              other.primitive_msg);
}

bool Intent::operator!=(const Intent &other) const
{
    return !((*this) == other);
}

void Intent::setMotionConstraints(const std::set<MotionConstraint> &motion_constraints)
{
    this->motion_constraints = motion_constraints;
}

std::set<MotionConstraint> Intent::getMotionConstraints(void) const
{
    return this->motion_constraints;
}

std::optional<NavigatorParams> Intent::getNavigatorParams() const
{
    return navigator_params;
}

PrimitiveMsg Intent::getPrimitiveMsg() const
{
    return primitive_msg;
}

PrimitiveMsg Intent::getUpdatedPrimitiveMsg(Point destination, double final_speed) const
{
    return primitive_msg;
}

void Intent::updateNavigatorParams(unsigned int robot_id, Point destination,
                                   Angle final_angle, double final_speed,
                                   BallCollisionType ball_collision_type)
{
    navigator_params = NavigatorParams{.destination         = destination,
                                       .final_speed         = final_speed,
                                       .final_angle         = final_angle,
                                       .ball_collision_type = ball_collision_type};
}
