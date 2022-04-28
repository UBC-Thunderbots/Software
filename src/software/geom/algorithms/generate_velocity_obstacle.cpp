#include "software/geom/algorithms/generate_velocity_obstacle.h"

#include "software/geom/geom_constants.h"

// TODO: Could add velocity aswell...
Agent::VelocityObstacle generateVelocityObstacle(const Circle &circle, const Agent& agent)
{
    Agent::VelocityObstacle velocity_obstacle;
    velocityObstacle.apex_ = circle.origin();

    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent

        const float angle =
                (position_ - other_agent.getPosition()).orientation().toRadians();

        // opening angle = arcsin((rad_A + rad_B) / distance) TODO: Double check this
        const float openingAngle =
                std::asin((agent.getRadius() + circle.radius()) /
                          (circle.origin() - agent.getPosition()).length());

        // Direction of the two edges of the velocity obstacle
        velocityObstacle.side1_ =
                Vector(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
        velocityObstacle.side2_ =
                Vector(std::cos(angle + openingAngle), std::sin(angle + openingAngle));
    }
    else
    {
        // This Agent is colliding with other agent
        // Creates Velocity Obstacle with the sides being 180 degrees
        // apart from each other
        velocityObstacle.side1_ =
                (agent.getPosition() - circle.origin()).perpendicular().normalize();
        velocityObstacle.side2_ = -velocityObstacle.side1_;
    }
    return velocity_obstacle;
}

Agent::VelocityObstacle generateVelocityObstacle(const Rectangle &rectangle, const Agent& agent)
{
    Agent::VelocityObstacle velocity_obstacle;

    return velocity_obstacle;
}

Agent::VelocityObstacle generateVelocityObstacle(const Polygon &polygon, const Agent& agent)
{
    Agent::VelocityObstacle velocity_obstacle;

    return velocity_obstacle;
}
