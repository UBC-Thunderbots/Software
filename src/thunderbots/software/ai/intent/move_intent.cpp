#include "ai/intent/move_intent.h"

#include "ai/intent/visitor/intent_visitor.h"
#include "util/logger/init.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, unsigned int priority, bool enable_dribbler,
                       AutokickType autokick)
    : MovePrimitive(robot_id, dest, final_angle, final_speed, enable_dribbler, autokick),
      Intent(priority),
      flags(MoveFlags::NONE),
      additional_obstacles({})
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void MoveIntent::setMoveFlags(MoveFlags flags)
{
    if (!isMoveFlagValid(flags))
    {
        LOG(WARNING) << "Invalid MoveFlags set" << std::endl;
    }
    this->flags = flags;
}

MoveFlags MoveIntent::getMoveFlags()
{
    return flags;
}

void MoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return MovePrimitive::operator==(other) && Intent::operator==(other);
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}

/*
 * Gets additional obstacles for this move intent
 *
 * @return additional obstacles for this move intent
 */
std::vector<Obstacle> MoveIntent::getAdditionalObstacles() const
{
    return additional_obstacles;
}

/*
 * Adds an additional obstaclesfor this move intent
 *
 * @param an additional obstacle for this move intent
 */
void MoveIntent::getAdditionalObstacles(Obstacle o)
{
    additional_obstacles.push_back(o);
}
